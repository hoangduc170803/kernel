package org.opentcs.fms.kernel.mapf;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.pibt.Footprint;

class ReplanPolicyTest {

  private static final Set<String> ALL_MOBILE = Set.of();
  private static final Map<String, String> AT_A = Map.of("V1", "A");
  private static final Map<String, String> AT_D_OFF_ROUTE = Map.of("V1", "D");
  private static final Map<String, String> GOAL_C = Map.of("V1", "C");
  private static final Map<String, String> GOAL_B = Map.of("V1", "B");
  private static final Map<String, MapfCoordinator.PlannedRoute> NO_PLAN = Map.of();
  private static final Map<String, MapfCoordinator.PlannedRoute> COMMITTED_A_B_C =
      Map.of("V1", new MapfCoordinator.PlannedRoute(List.of("A", "B", "C"), "C", "C"));

  @Test
  void theThrottleCoalescesReSolvesWithinTheWindow() {
    ReplanPolicy policy = solvedAt(0L, AT_A, GOAL_C);

    assertThat(policy.shouldSolve(AT_A, GOAL_B, ALL_MOBILE, COMMITTED_A_B_C, 500L))
        .as("the goal changed, but 500ms after the last solve is inside the 1000ms gap")
        .isFalse();
    assertThat(policy.shouldSolve(AT_A, GOAL_B, ALL_MOBILE, COMMITTED_A_B_C, 1_500L))
        .as("and the coalesced change is taken as soon as the window has passed. This is a"
            + " THROTTLE, not a debounce: a flood of changes never pushes the deadline out,"
            + " because the window is measured from the last SOLVE, not the last change")
        .isTrue();
  }

  @Test
  void commitHorizonForcesReSolveWithNoChange() {
    ReplanPolicy policy = solvedAt(0L, AT_A, GOAL_C);

    assertThat(policy.shouldSolve(AT_A, GOAL_C, ALL_MOBILE, COMMITTED_A_B_C, 4_000L))
        .as("nothing about the fleet changed, but the 3500ms commit horizon has elapsed")
        .isTrue();
  }

  @Test
  void overdueTurnsTrueOnceTheCommitHorizonElapses() {
    ReplanPolicy policy = new ReplanPolicy();

    assertThat(policy.overdueAt(0L, NO_PLAN))
        .as("before the first solve there is no committed plan that could go stale")
        .isFalse();

    assertThat(policy.shouldSolve(AT_A, GOAL_C, ALL_MOBILE, NO_PLAN, 0L)).isTrue();
    policy.recordSolveEnded(0L);

    assertThat(policy.overdueAt(3_000L, COMMITTED_A_B_C)).isFalse();
    assertThat(policy.overdueAt(3_500L, COMMITTED_A_B_C)).isTrue();

    assertThat(policy.shouldSolve(AT_A, GOAL_C, ALL_MOBILE, COMMITTED_A_B_C, 3_500L)).isTrue();
    policy.recordSolveEnded(3_500L);
    assertThat(policy.overdueAt(3_500L, COMMITTED_A_B_C))
        .as("re-solving restarts the horizon from the moment that solve ENDED")
        .isFalse();
  }

  @Test
  void reSolvesWhenAVehicleLeavesItsCommittedRoute() {
    ReplanPolicy policy = solvedAt(0L, AT_A, GOAL_C);

    assertThat(policy.shouldSolve(AT_D_OFF_ROUTE, GOAL_C, ALL_MOBILE, COMMITTED_A_B_C, 2_000L))
        .as("D is not on the committed A,B,C route, so the plan no longer describes the fleet"
            + " and holding it would steer from a position nobody planned for")
        .isTrue();
  }

  @Test
  void reSolvesWhenAGoalChanges() {
    ReplanPolicy policy = solvedAt(0L, AT_A, GOAL_C);

    assertThat(policy.shouldSolve(AT_A, GOAL_B, ALL_MOBILE, COMMITTED_A_B_C, 2_000L))
        .as("the fleet was solved for C and is now asked for B")
        .isTrue();
  }

  @Test
  void commitsThePlanWhenNothingChanges() {
    ReplanPolicy policy = solvedAt(0L, AT_A, GOAL_C);

    assertThat(policy.shouldSolve(AT_A, GOAL_C, ALL_MOBILE, COMMITTED_A_B_C, 2_000L))
        .as("same goals and V1 still standing on its committed route, past the throttle window and"
            + " short of the commit horizon: the plan stands")
        .isFalse();
  }

  @Test
  void theThrottleWindowStartsWhenTheSolveENDSNotWhenItBegins() {
    ReplanPolicy policy = new ReplanPolicy();

    assertThat(policy.shouldSolve(AT_A, GOAL_C, ALL_MOBILE, NO_PLAN, 0L)).isTrue();
    policy.recordSolveEnded(900L);
    policy.recordSolved(GOAL_C, ALL_MOBILE);

    assertThat(policy.shouldSolve(AT_A, GOAL_B, ALL_MOBILE, COMMITTED_A_B_C, 1_500L))
        .as("1500ms after the solve STARTED, but only 600ms after it ENDED: still inside"
            + " the 1000ms window, so the solver is guaranteed a rest between two runs")
        .isFalse();
    assertThat(policy.shouldSolve(AT_A, GOAL_B, ALL_MOBILE, COMMITTED_A_B_C, 1_900L))
        .as("1000ms after it ended, the next solve is allowed")
        .isTrue();
  }

  @Test
  void aSolveThatProducedNoUsablePlanStillOpensTheThrottleWindow() {
    ReplanPolicy policy = new ReplanPolicy();

    assertThat(policy.shouldSolve(AT_A, GOAL_C, ALL_MOBILE, NO_PLAN, 0L)).isTrue();
    policy.recordSolveEnded(800L);

    assertThat(policy.shouldSolve(AT_A, GOAL_C, ALL_MOBILE, COMMITTED_A_B_C, 1_000L))
        .as("the solve burnt 800ms of CPU and returned nothing usable; re-solving immediately"
            + " would saturate the solver, so the window applies to failures too")
        .isFalse();
  }

  @Test
  void cargoAppearingOrLeavingForcesAReSolve() {
    ReplanPolicy policy = solvedAt(0L, AT_A, GOAL_C);
    List<Footprint.Obstacle> cargo = List.of(new Footprint.Obstacle("cargo", 0, 800, 0, 740));

    assertThat(policy.shouldSolve(AT_A, GOAL_C, ALL_MOBILE, COMMITTED_A_B_C, 1_500L))
        .isFalse();

    policy.observeObstacles(cargo);
    assertThat(policy.shouldSolve(AT_A, GOAL_C, ALL_MOBILE, COMMITTED_A_B_C, 1_500L))
        .isTrue();

    policy.recordSolveEnded(1_500L);
    policy.recordSolved(GOAL_C, ALL_MOBILE, cargo);
    policy.observeObstacles(List.of());
    assertThat(policy.shouldSolve(AT_A, GOAL_C, ALL_MOBILE, COMMITTED_A_B_C, 2_500L))
        .isTrue();
  }

  private static ReplanPolicy solvedAt(
      long now, Map<String, String> positions, Map<String, String> goals) {
    ReplanPolicy policy = new ReplanPolicy();
    policy.shouldSolve(positions, goals, ALL_MOBILE, NO_PLAN, now);
    policy.recordSolveEnded(now);
    policy.recordSolved(goals, ALL_MOBILE);
    return policy;
  }
}
