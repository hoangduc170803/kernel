package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class Lacam2SolverTest {

  @BeforeAll
  static void theseFixturesMeasureInGraphUnitsNotMillimetres() {
    System.setProperty("fms.pibt.footprint", "false");
  }

  @AfterAll
  static void restoreFootprintForEveryoneElse() {
    System.clearProperty("fms.pibt.footprint");
  }

  private static final long DEADLINE_MS = 5_000L;
  private static final int SEED = 0;
  private static final int WINDOW = 3;
  private static final int WINDOWED_LOOP_LIMIT = 5_000;

  @Test
  void solvesSixAgentsCrossingOnAFourByThreeGrid() {
    Roadmap grid = crossingGrid();
    int[] starts = crossingStarts(grid);
    int[] goals = crossingGoals(grid);

    Lacam2Solver.Result result = solve(grid, starts, goals, Lacam2Solver.Objective.NONE);

    assertThat(result.solved()).isTrue();
    assertValidPlan(result, starts, goals);
  }

  @Test
  void aWindowedPlanStopsAtTheWindowAndStaysConflictFree() {
    Roadmap grid = crossingGrid();
    int[] starts = crossingStarts(grid);
    int[] goals = crossingGoals(grid);

    Lacam2Solver.Result result = solveWindowed(grid, starts, goals, WINDOW);

    assertThat(result.solved()).isTrue();
    assertThat(result.reachedGoals())
        .as("the window cuts the search short of the goal configuration")
        .isFalse();
    assertThat(result.solution().get(0)).containsExactly(starts);
    assertThat(makespanOf(result)).isLessThanOrEqualTo(WINDOW + 1);
    assertThat(conflictsIn(result.solution())).isEmpty();
  }

  @Test
  void reachingTheWindowEdgeDoesNotArmTheFPruneAndCollapseTheSearch() {
    Roadmap grid = crossingGrid();
    int[] starts = crossingStarts(grid);
    int[] goals = crossingGoals(grid);

    Lacam2Solver.Result result = solveWindowed(grid, starts, goals, WINDOW);

    assertThat(result.loopCount())
        .as("a window edge is where the search stops looking, not a solution; arming the f-prune"
            + " against it discards every equal-f alternative and collapses the search to the"
            + " first PIBT rollout")
        .isGreaterThan(100);
  }

  @Test
  void reportsNoSolutionForAHeadOnSwapInASingleLaneCorridor() {
    Roadmap corridor = GridFixture.grid(5, 1);
    int[] starts = { GridFixture.id(corridor, 0, 0), GridFixture.id(corridor, 0, 4) };
    int[] goals = { GridFixture.id(corridor, 0, 4), GridFixture.id(corridor, 0, 0) };

    Lacam2Solver.Result result = solve(corridor, starts, goals, Lacam2Solver.Objective.NONE);

    assertThat(result.solved()).isFalse();
    assertThat(result.solution()).isEmpty();
    assertThat(result.runtimeMs())
        .as("the lane has no passing room, so the search must exhaust rather than hang")
        .isLessThan(DEADLINE_MS);
  }

  @Test
  void aCommittedBackStepIsExecutedEvenWhenTheGuideWouldGoForward() {
    Roadmap corridor = GridFixture.grid(4, 1);
    int start = GridFixture.id(corridor, 0, 1);
    int goal = GridFixture.id(corridor, 0, 0);
    int committed = GridFixture.id(corridor, 0, 2);
    Lacam2Solver.Result withoutCommitment = solveCommitted(
        corridor, new int[] { start }, new int[] { goal }, new int[][] { new int[0] });

    Lacam2Solver.Result result = solveCommitted(
        corridor, new int[] { start }, new int[] { goal },
        new int[][] { { start, committed } });

    assertThat(withoutCommitment.solution().get(1)[0]).isEqualTo(goal);
    assertThat(result.solved()).isTrue();
    assertThat(result.solution().get(1)[0]).isEqualTo(committed);
    assertThat(conflictsIn(result.solution())).isEmpty();
  }

  @Test
  void aCommittedTurnIsPinnedEvenWhileTheAgentIsRolling() {
    Roadmap grid = GridFixture.grid(3, 3);
    int center = GridFixture.id(grid, 1, 1);
    int west = GridFixture.id(grid, 1, 0);
    int south = GridFixture.id(grid, 2, 1);
    int east = GridFixture.id(grid, 1, 2);
    int[] starts = { center };
    int[] goals = { east };
    int[] startMotion = { grid.motionAfter(west, center) };

    Lacam2Solver.Result result = solveCommittedRolling(
        grid, starts, goals, new int[][] { { center, south } }, startMotion);

    assertThat(result.solved()).isTrue();
    assertThat(result.solution().get(1)[0]).isEqualTo(south);
    assertThat(conflictsIn(result.solution())).isEmpty();
  }

  @Test
  void aCommittedReversalIsPinnedEvenWhileTheAgentIsRolling() {
    Roadmap corridor = GridFixture.grid(4, 1);
    int start = GridFixture.id(corridor, 0, 1);
    int west = GridFixture.id(corridor, 0, 0);
    int goal = GridFixture.id(corridor, 0, 3);
    int[] starts = { start };
    int[] goals = { goal };
    int[] startMotion = { corridor.motionAfter(west, start) };

    Lacam2Solver.Result result = solveCommittedRolling(
        corridor, starts, goals, new int[][] { { start, west } }, startMotion);

    assertThat(result.solved()).isTrue();
    assertThat(result.solution().get(1)[0]).isEqualTo(west);
  }

  @Test
  void aCommittedTurnAfterAStraightStepIsPinnedAtDepthOne() {
    Roadmap grid = GridFixture.grid(3, 3);
    int start = GridFixture.id(grid, 1, 0);
    int straightStep = GridFixture.id(grid, 1, 1);
    int turn = GridFixture.id(grid, 0, 1);
    int goal = GridFixture.id(grid, 1, 2);

    Lacam2Solver.Result result = solveCommittedRolling(
        grid,
        new int[] { start },
        new int[] { goal },
        new int[][] { { start, straightStep, turn } },
        new int[] { Roadmap.DIR_NONE });

    assertThat(result.solved()).isTrue();
    assertThat(result.solution().get(1)[0]).isEqualTo(straightStep);
    assertThat(result.solution().get(2)[0]).isEqualTo(turn);
  }

  @Test
  void otherAgentsPlanAroundACommittedStepIntoAPocket() {
    Roadmap grid = GridFixture.grid(5, 2);
    int pocketEntry = GridFixture.id(grid, 0, 1);
    int pocket = GridFixture.id(grid, 1, 1);
    int vacatedCell = pocketEntry;
    int[] starts = { pocketEntry, GridFixture.id(grid, 0, 2) };
    int[] goals = { GridFixture.id(grid, 0, 4), GridFixture.id(grid, 0, 0) };

    Lacam2Solver.Result result = solveCommitted(
        grid, starts, goals, new int[][] { { pocketEntry, pocket }, new int[0] });

    assertThat(result.solved()).isTrue();
    assertThat(result.solution().get(1)[0]).isEqualTo(pocket);
    assertThat(result.solution().stream().map(config -> config[1]))
        .contains(vacatedCell)
        .last()
        .isEqualTo(goals[1]);
    assertThat(conflictsIn(result.solution())).isEmpty();
  }

  @Test
  void aCommittedTargetOccupiedByAStandingAgentIsNotForced() {
    Roadmap grid = GridFixture.grid(3, 2);
    int aStart = GridFixture.id(grid, 0, 0);
    int bStart = GridFixture.id(grid, 0, 1);
    int[] starts = { aStart, bStart };
    int[] goals = { GridFixture.id(grid, 1, 2), bStart };

    Lacam2Solver.Result result = solveCommitted(
        grid, starts, goals, new int[][] { { aStart, bStart }, { bStart } });

    assertThat(result.solved()).isTrue();
    assertThat(result.solution().get(1)[0]).isNotEqualTo(bStart);
    assertThat(conflictsIn(result.solution())).isEmpty();
  }

  @Test
  void twoCommitmentsOnTheSameCellPinOnlyOne() {
    Roadmap grid = GridFixture.grid(3, 3);
    int center = GridFixture.id(grid, 1, 1);
    int[] starts = { GridFixture.id(grid, 1, 0), GridFixture.id(grid, 0, 1) };
    int[] goals = { GridFixture.id(grid, 1, 2), GridFixture.id(grid, 2, 1) };

    Lacam2Solver.Result result = solveCommitted(
        grid, starts, goals,
        new int[][] { { starts[0], center }, { starts[1], center } });

    assertThat(result.solved()).isTrue();
    assertThat(conflictsIn(result.solution())).isEmpty();
    assertThat((result.solution().get(1)[0] == center ? 1 : 0)
        + (result.solution().get(1)[1] == center ? 1 : 0)).isEqualTo(1);
  }

  @Test
  void withoutCommittedPrefixesThePlanIsUnchanged() {
    Roadmap corridor = GridFixture.grid(4, 1);
    MapfInstance instance = new MapfInstance(
        new int[] { GridFixture.id(corridor, 0, 0) },
        new int[] { GridFixture.id(corridor, 0, 3) });
    Lacam2Solver solver = new Lacam2Solver(corridor);
    Lacam2Solver.Result emptyPrefixes = solver.solve(
        instance, DEADLINE_MS, SEED, Lacam2Solver.Objective.NONE,
        Lacam2Solver.DEFAULT_RESTART_RATE, Lacam2Solver.LOOP_LIMIT_UNBOUNDED,
        SolverParams.WINDOW_FULL, null, null, null, null, new int[][] { new int[0] },
        StaticBodies.NONE);
    Lacam2Solver.Result noPrefixes = solver.solve(
        instance, DEADLINE_MS, SEED, Lacam2Solver.Objective.NONE,
        Lacam2Solver.DEFAULT_RESTART_RATE, Lacam2Solver.LOOP_LIMIT_UNBOUNDED,
        SolverParams.WINDOW_FULL, null, null, null, null, null, StaticBodies.NONE);

    assertThat(emptyPrefixes.solved()).isEqualTo(noPrefixes.solved());
    assertThat(emptyPrefixes.solution()).hasSameSizeAs(noPrefixes.solution());
    for (int depth = 0; depth < emptyPrefixes.solution().size(); depth++) {
      assertThat(emptyPrefixes.solution().get(depth))
          .containsExactly(noPrefixes.solution().get(depth));
    }
  }

  @Test
  void aTwoStepCommitmentIsForcedAtBothDepths() {
    Roadmap corridor = GridFixture.grid(5, 1);
    int start = GridFixture.id(corridor, 0, 2);
    int firstCommitment = GridFixture.id(corridor, 0, 1);
    int secondCommitment = GridFixture.id(corridor, 0, 0);
    int goal = GridFixture.id(corridor, 0, 4);

    Lacam2Solver.Result result = solveCommitted(
        corridor, new int[] { start }, new int[] { goal },
        new int[][] { { start, firstCommitment, secondCommitment } });

    assertThat(result.solved()).isTrue();
    assertThat(result.solution().get(1)[0]).isEqualTo(firstCommitment);
    assertThat(result.solution().get(2)[0]).isEqualTo(secondCommitment);
  }

  @Test
  void reportsNoSolutionWhenTheGoalLiesInADisconnectedComponent() {
    Roadmap split = twoIslands();
    int[] starts = idsOf(split, "west0");
    int[] goals = idsOf(split, "east1");

    Lacam2Solver.Result result = solve(split, starts, goals, Lacam2Solver.Objective.NONE);

    assertThat(result.solved())
        .as("no path exists, so the clamped infinite distance must not fake a gradient to the goal")
        .isFalse();
    assertThat(result.optimal()).isFalse();
    assertThat(result.runtimeMs()).isLessThan(DEADLINE_MS);
  }

  @Test
  void constraintCandidatesPutPlainPointsAheadOfCargoSlots() {
    Roadmap map = hubWithCargoSlots();
    int[] candidates = {
        map.idOf("c1"), map.idOf("n1"), map.idOf("c2"), map.idOf("n2") };

    Lacam2Solver.sortByVertexWeight(candidates, map);

    assertThat(candidates)
        .as("plain points first, and the shuffled order inside each weight class is preserved")
        .containsExactly(map.idOf("n1"), map.idOf("n2"), map.idOf("c1"), map.idOf("c2"));
  }

  @Test
  void holdingPositionOnACargoSlotSortsBehindEveryPlainNeighbour() {
    Roadmap map = hubWithCargoSlots();
    int[] candidates = map.candidates(map.idOf("c1"));

    Lacam2Solver.sortByVertexWeight(candidates, map);

    assertThat(candidates[candidates.length - 1])
        .as("staying put keeps occupying the slot, so it ranks with the slots, not with the moves")
        .isEqualTo(map.idOf("c1"));
  }

  @Test
  void exhaustingTheSearchExercisesTheLowLevelConstraintTree() {
    Roadmap corridor = GridFixture.grid(5, 1);
    int[] starts = { GridFixture.id(corridor, 0, 0), GridFixture.id(corridor, 0, 4) };
    int[] goals = { GridFixture.id(corridor, 0, 4), GridFixture.id(corridor, 0, 0) };

    Lacam2Solver.Result result = solve(corridor, starts, goals, Lacam2Solver.Objective.NONE);

    assertThat(result.constrainedCalls())
        .as("a drained OPEN means every LowNode was consumed, so constrained PIBT calls ran")
        .isGreaterThan(0);
    assertThat(result.constrainedCalls()).isLessThanOrEqualTo(result.loopCount());
  }

  @Test
  void aBoostedAgentOutranksAFartherUnboostedOneAtEveryFleetSize() {
    Roadmap lane = GridFixture.grid(40, 1);
    int[] starts = { GridFixture.id(lane, 0, 1), GridFixture.id(lane, 0, 38) };
    int[] goals = { GridFixture.id(lane, 0, 0), GridFixture.id(lane, 0, 0) };
    AgentPriority[] starvingIsNearItsGoal = {
        new AgentPriority(0, 0, 1, 0, 0f), new AgentPriority(0, 0, 0, 0, 0f) };

    Lacam2Solver.Result result = new Lacam2Solver(lane).solve(
        new MapfInstance(starts, goals), DEADLINE_MS, SEED,
        Lacam2Solver.Objective.SUM_OF_LOSS, Lacam2Solver.DEFAULT_RESTART_RATE,
        WINDOWED_LOOP_LIMIT, WINDOW, starvingIsNearItsGoal,
        null, null, null, null, StaticBodies.NONE);

    assertThat(result.solved()).isTrue();
    assertThat(result.solution().get(1)[0])
        .as("one tick of starvation must outrank a 37-hop distance gap, or the boost is inert")
        .isEqualTo(goals[0]);
  }

  private static Roadmap hubWithCargoSlots() {
    List<Roadmap.Point> points = List.of(
        new Roadmap.Point("hub", 0, 0, 0),
        new Roadmap.Point("n1", 0, 1, 0),
        new Roadmap.Point("n2", -1, 0, 0),
        new Roadmap.Point("c1", 1, 0, 0),
        new Roadmap.Point("c2", 0, -1, 0));
    List<Roadmap.Path> paths = List.of(
        new Roadmap.Path("hub-n1", "hub", "n1", 1, true),
        new Roadmap.Path("hub-n2", "hub", "n2", 1, true),
        new Roadmap.Path("hub-c1", "hub", "c1", 1, true),
        new Roadmap.Path("hub-c2", "hub", "c2", 1, true));
    return new Roadmap(new Roadmap.Model(points, paths, Set.of("c1", "c2")));
  }

  private static Roadmap crossingGrid() {
    return GridFixture.grid(4, 3);
  }

  private static int[] crossingStarts(Roadmap grid) {
    return new int[] {
        GridFixture.id(grid, 0, 0), GridFixture.id(grid, 0, 1), GridFixture.id(grid, 0, 2),
        GridFixture.id(grid, 2, 0), GridFixture.id(grid, 2, 1), GridFixture.id(grid, 2, 2) };
  }

  private static int[] crossingGoals(Roadmap grid) {
    return new int[] {
        GridFixture.id(grid, 2, 3), GridFixture.id(grid, 2, 2), GridFixture.id(grid, 2, 1),
        GridFixture.id(grid, 0, 3), GridFixture.id(grid, 0, 2), GridFixture.id(grid, 0, 1) };
  }

  private static Roadmap twoIslands() {
    List<Roadmap.Point> points = List.of(
        new Roadmap.Point("west0", 0, 0, 0),
        new Roadmap.Point("west1", 1, 0, 0),
        new Roadmap.Point("east0", 9, 0, 0),
        new Roadmap.Point("east1", 10, 0, 0));
    List<Roadmap.Path> paths = List.of(
        new Roadmap.Path("west0->west1", "west0", "west1", 1, true),
        new Roadmap.Path("east0->east1", "east0", "east1", 1, true));
    return new Roadmap(new Roadmap.Model(points, paths));
  }

  @Test
  void agedAccumulatesAcrossSolvesSoStarvationOutlivesASingleCycle() {
    Roadmap corridor = GridFixture.grid(5, 1);
    int[] starts = new int[] { GridFixture.id(corridor, 0, 0) };
    int[] goals = new int[] { GridFixture.id(corridor, 0, 4) };

    Lacam2Solver.Result first = solveSeeded(corridor, starts, goals, null, 1);
    assertThat(first.executedPriorities()[0].aged())
        .as("one tick away from the goal and still not on it, so the vehicle has aged once")
        .isEqualTo(1);

    Lacam2Solver.Result second = solveSeeded(
        corridor, starts, goals, first.executedPriorities(), 1);
    assertThat(second.executedPriorities()[0].aged())
        .as("feeding the remembered priority back in has to carry the count forward rather than"
            + " restart it; a starvation counter that resets every cycle cannot outrank anything")
        .isEqualTo(2);
  }

  @Test
  void theRememberedStarvationDecidesWhoGetsTheContestedCell() {
    Roadmap cross = GridFixture.grid(3, 3);
    int junction = GridFixture.id(cross, 1, 1);
    int westArm = GridFixture.id(cross, 1, 0);
    int eastArm = GridFixture.id(cross, 1, 2);
    int northArm = GridFixture.id(cross, 0, 1);
    int southArm = GridFixture.id(cross, 2, 1);
    int[] starts = new int[] { westArm, northArm };
    int[] goals = new int[] { eastArm, southArm };

    Lacam2Solver.Result westIsStarving =
        solveSeeded(cross, starts, goals, seed(9, 0), 1);
    Lacam2Solver.Result northIsStarving =
        solveSeeded(cross, starts, goals, seed(0, 9), 1);

    assertThat(westIsStarving.solution().get(1)[0])
        .as("two vehicles cross at a junction on their way to goals that are their own and lie in"
            + " arms neither of them starts in, so neither is in the other's way and only the"
            + " ordering can say who takes the junction first. Nine remembered ticks of"
            + " starvation on the westbound vehicle has to win it")
        .isEqualTo(junction);
    assertThat(northIsStarving.solution().get(1)[1])
        .as("and moving the starvation to the other vehicle has to hand the junction the other"
            + " way; if the remembered priority never reached the ordering these two runs could"
            + " not disagree")
        .isEqualTo(junction);
  }

  @Test
  void aVehicleAlreadyInTheGoalsLaneOutranksOneThatWouldHaveToCrossIntoIt() {
    Roadmap lane = GridFixture.grid(3, 2);
    int aboveGoal = GridFixture.id(lane, 0, 1);
    int besideGoal = GridFixture.id(lane, 1, 2);
    int goal = GridFixture.id(lane, 0, 2);
    int[] starts = new int[] { aboveGoal, besideGoal };
    int[] goals = new int[] { goal, goal };

    AgentPriority inLane = new AgentPriority(0, 1, 0, 0, 0f);
    AgentPriority crossingIn = new AgentPriority(0, 0, 5, 0, 0f);

    Lacam2Solver.Result result = solveSeeded(
        lane, starts, goals, new AgentPriority[] { inLane, crossingIn }, 1);

    assertThat(result.solution().get(1)[0])
        .as("both vehicles are one hop from the same cell, and the one that would have to cross"
            + " lanes carries five ticks of starvation against none. Standing in the goal's own"
            + " lane has to outrank that, otherwise the tier is below aged and inert")
        .isEqualTo(goal);
  }

  @Test
  void sharesLaneWithFollowsTheColumnRatherThanJustMatchingACoordinate() {
    Roadmap lane = GridFixture.grid(3, 3);
    int top = GridFixture.id(lane, 0, 1);
    int bottom = GridFixture.id(lane, 2, 1);
    int sideways = GridFixture.id(lane, 0, 2);

    assertThat(lane.sharesLaneWith(top, bottom))
        .as("a straight unbroken column counts as one lane end to end")
        .isTrue();
    assertThat(lane.sharesLaneWith(top, sideways))
        .as("a lane is not only vertical: two cells sharing a row are one lane as well")
        .isTrue();
    assertThat(lane.sharesLaneWith(top, GridFixture.id(lane, 2, 2)))
        .as("a cell that shares neither coordinate cannot be reached without leaving the lane")
        .isFalse();
  }

  private static AgentPriority[] seed(int agedLeft, int agedRight) {
    return new AgentPriority[] {
        new AgentPriority(0, 0, agedLeft, 0, 0f),
        new AgentPriority(0, 0, agedRight, 0, 0f) };
  }

  private static Lacam2Solver.Result solveSeeded(
      Roadmap roadmap, int[] starts, int[] goals, AgentPriority[] rootPriorities, int window) {
    return new Lacam2Solver(roadmap).solve(
        new MapfInstance(starts, goals),
        DEADLINE_MS,
        SEED,
        Lacam2Solver.Objective.SUM_OF_LOSS,
        Lacam2Solver.DEFAULT_RESTART_RATE,
        WINDOWED_LOOP_LIMIT,
        window,
        rootPriorities,
        null,
        null,
        null,
        null,
        StaticBodies.NONE);
  }

  private static Lacam2Solver.Result solve(
      Roadmap roadmap, int[] starts, int[] goals, Lacam2Solver.Objective objective) {
    return new Lacam2Solver(roadmap).solve(
        new MapfInstance(starts, goals),
        DEADLINE_MS,
        SEED,
        objective,
        Lacam2Solver.DEFAULT_RESTART_RATE,
        Lacam2Solver.LOOP_LIMIT_UNBOUNDED,
        SolverParams.WINDOW_FULL,
        null,
        null,
        null,
        null,
        null,
        StaticBodies.NONE);
  }

  private static Lacam2Solver.Result solveCommitted(
      Roadmap roadmap, int[] starts, int[] goals, int[][] committedPrefixes) {
    return new Lacam2Solver(roadmap).solve(
        new MapfInstance(starts, goals),
        DEADLINE_MS,
        SEED,
        Lacam2Solver.Objective.NONE,
        Lacam2Solver.DEFAULT_RESTART_RATE,
        Lacam2Solver.LOOP_LIMIT_UNBOUNDED,
        SolverParams.WINDOW_FULL,
        null,
        null,
        null,
        null,
        committedPrefixes,
        StaticBodies.NONE);
  }

  private static Lacam2Solver.Result solveCommittedRolling(
      Roadmap roadmap, int[] starts, int[] goals, int[][] committedPrefixes, int[] startMotion) {
    return new Lacam2Solver(roadmap).solve(
        new MapfInstance(starts, goals),
        DEADLINE_MS,
        SEED,
        Lacam2Solver.Objective.NONE,
        Lacam2Solver.DEFAULT_RESTART_RATE,
        Lacam2Solver.LOOP_LIMIT_UNBOUNDED,
        SolverParams.WINDOW_FULL,
        null,
        null,
        null,
        null,
        committedPrefixes,
        StaticBodies.NONE,
        startMotion,
        new LegalMoves(roadmap));
  }

  private static Lacam2Solver.Result solveWindowed(
      Roadmap roadmap, int[] starts, int[] goals, int window) {
    return new Lacam2Solver(roadmap).solve(
        new MapfInstance(starts, goals),
        DEADLINE_MS,
        SEED,
        Lacam2Solver.Objective.SUM_OF_LOSS,
        Lacam2Solver.DEFAULT_RESTART_RATE,
        WINDOWED_LOOP_LIMIT,
        window,
        null,
        null,
        null,
        null,
        null,
        StaticBodies.NONE);
  }

  private static int[] idsOf(Roadmap roadmap, String... names) {
    int[] ids = new int[names.length];
    for (int i = 0; i < names.length; i++) {
      ids[i] = roadmap.idOf(names[i]);
    }
    return ids;
  }

  private static int makespanOf(Lacam2Solver.Result result) {
    return result.solution().size() - 1;
  }

  private static void assertValidPlan(Lacam2Solver.Result result, int[] starts, int[] goals) {
    List<int[]> plan = result.solution();
    assertThat(plan).isNotEmpty();
    assertThat(plan.get(0)).as("plan must begin at the starts").containsExactly(starts);
    assertThat(plan.get(plan.size() - 1)).as("plan must end at the goals").containsExactly(goals);
    assertThat(conflictsIn(plan)).isEmpty();
  }

  private static List<String> conflictsIn(List<int[]> plan) {
    List<String> conflicts = new ArrayList<>();
    for (int t = 0; t < plan.size(); t++) {
      int[] current = plan.get(t);
      for (int a = 0; a < current.length; a++) {
        for (int b = a + 1; b < current.length; b++) {
          if (current[a] == current[b]) {
            conflicts.add("vertex conflict @t=" + t + " v=" + current[a]
                + " agents " + a + "," + b);
          }
          if (t > 0 && swaps(plan.get(t - 1), current, a, b)) {
            conflicts.add("swap conflict @t=" + t + " agents " + a + "," + b);
          }
        }
      }
    }
    return conflicts;
  }

  private static boolean swaps(int[] before, int[] after, int a, int b) {
    return after[a] == before[b] && after[b] == before[a] && before[a] != after[a];
  }

  // --- kinematics: the emitted plan obeys the brake tick -----------------------

  private static final int KINEMATIC_WINDOW = 12;

  private static final int KINEMATIC_SEEDS = 8;

  private static Roadmap kinematicGrid() {
    return GridFixture.grid(6, 6);
  }

  private static MapfInstance kinematicInstance(Roadmap map) {
    return new MapfInstance(
        new int[] {GridFixture.id(map, 0, 0), GridFixture.id(map, 5, 5)},
        new int[] {GridFixture.id(map, 4, 3), GridFixture.id(map, 1, 2)});
  }

  private static Lacam2Solver.Result solveKinematic(
      Roadmap map, MapfInstance instance, int seed) {
    return new Lacam2Solver(map).solve(
        instance, 2_000L, seed, Lacam2Solver.Objective.NONE,
        Lacam2Solver.DEFAULT_RESTART_RATE, Lacam2Solver.LOOP_LIMIT_UNBOUNDED, KINEMATIC_WINDOW,
        null, null, null, null, null, StaticBodies.NONE);
  }

  private static int directionChangesWithoutBraking(Roadmap map, List<int[]> plan, int agent) {
    int offences = 0;
    int motion = Roadmap.DIR_NONE;
    for (int t = 0; t + 1 < plan.size(); t++) {
      int next = map.motionAfter(plan.get(t)[agent], plan.get(t + 1)[agent]);
      if (motion != Roadmap.DIR_NONE && next != Roadmap.DIR_NONE && next != motion) {
        offences++;
      }
      motion = next;
    }
    return offences;
  }

  private static int brakeTicksBeforeATurn(Roadmap map, List<int[]> plan, int agent) {
    int paid = 0;
    int motion = Roadmap.DIR_NONE;
    boolean stoodStill = false;
    for (int t = 0; t + 1 < plan.size(); t++) {
      int next = map.motionAfter(plan.get(t)[agent], plan.get(t + 1)[agent]);
      if (next == Roadmap.DIR_NONE) {
        stoodStill = motion != Roadmap.DIR_NONE || stoodStill;
      } else {
        if (stoodStill && motion != Roadmap.DIR_NONE && next != motion) {
          paid++;
        }
        stoodStill = false;
        motion = next;
      }
    }
    return paid;
  }

  @Test
  void aPlanNeverTurnsOrReversesWithoutStandingStillFirst() {
    Roadmap map = kinematicGrid();
    MapfInstance instance = kinematicInstance(map);

    for (int seed = 0; seed < KINEMATIC_SEEDS; seed++) {
      Lacam2Solver.Result result = solveKinematic(map, instance, seed);
      assertThat(result.solved()).as("seed %d", seed).isTrue();
      for (int agent = 0; agent < 2; agent++) {
        assertThat(directionChangesWithoutBraking(map, result.solution(), agent))
            .as("seed %d agent %d changes travel direction while still rolling", seed, agent)
            .isZero();
      }
    }
  }

  @Test
  void theseRoutesDoCornerSoTheGuardAboveIsNotVacuous() {
    Roadmap map = kinematicGrid();
    MapfInstance instance = kinematicInstance(map);

    int cornersPaidFor = 0;
    for (int seed = 0; seed < KINEMATIC_SEEDS; seed++) {
      Lacam2Solver.Result result = solveKinematic(map, instance, seed);
      assertThat(result.solved()).isTrue();
      for (int agent = 0; agent < 2; agent++) {
        cornersPaidFor += brakeTicksBeforeATurn(map, result.solution(), agent);
      }
    }

    assertThat(cornersPaidFor)
        .as("both agents have to corner to reach their goals, so the plans must contain brake"
            + " ticks; a run with zero would mean the guard above passes only because nothing"
            + " ever tried to turn")
        .isPositive();
  }
}
