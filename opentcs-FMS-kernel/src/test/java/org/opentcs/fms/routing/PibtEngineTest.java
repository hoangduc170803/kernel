package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.pibt.PibtEngine;

class PibtEngineTest {

  @BeforeAll
  static void theseFixturesMeasureInGraphUnitsNotMillimetres() {
    System.setProperty("fms.pibt.footprint", "false");
  }

  @AfterAll
  static void restoreFootprintForEveryoneElse() {
    System.clearProperty("fms.pibt.footprint");
  }

  @Test
  void stepIsCollisionFree() {
    Roadmap r = GridFixture.grid(3, 3);
    int[] starts = { GridFixture.id(r, 0, 0), GridFixture.id(r, 0, 2) };
    int[] goals = { GridFixture.id(r, 2, 2), GridFixture.id(r, 2, 0) };
    DistTable dist = DistTable.weighted(r, goals);

    PibtEngine pibt = new PibtEngine(r, 2, 1, dist);
    int[] next = pibt.step(starts, new int[] { 0, 1 }, List.of());

    assertThat(next).isNotNull();
    int[][] plan = { { starts[0], next[0] }, { starts[1], next[1] } };
    assertThat(GridFixture.findCollisions(plan)).isEmpty();
    assertThat(GridFixture.findIllegalMoves(r, plan)).isEmpty();
  }

  @Test
  void windowedReachesGoalsOnOpenGrid() {
    Roadmap r = GridFixture.grid(4, 4);
    int[] starts = { GridFixture.id(r, 0, 0), GridFixture.id(r, 0, 3) };
    int[] goals = { GridFixture.id(r, 3, 3), GridFixture.id(r, 3, 0) };
    PibtEngine pibt = new PibtEngine(r, 2, 7, DistTable.weighted(r, goals));
    int[][] paths = GridFixture.rollout(pibt, r, starts, goals, 64);

    assertThat(GridFixture.findCollisions(paths)).isEmpty();
    assertThat(GridFixture.findIllegalMoves(r, paths)).isEmpty();
    assertThat(paths[0][paths[0].length - 1]).isEqualTo(goals[0]);
    assertThat(paths[1][paths[1].length - 1]).isEqualTo(goals[1]);
  }

  @Test
  void doesNotEnterTheCellTheLeaderVacatesInTheSameTick() {
    Roadmap map = GridFixture.grid(3, 1);
    int followerStart = GridFixture.id(map, 0, 0);
    int leaderStart = GridFixture.id(map, 0, 1);
    int goal = GridFixture.id(map, 0, 2);
    int[] starts = {followerStart, leaderStart};

    int[] next = new PibtEngine(map, 2, 7, DistTable.weighted(map, new int[] {goal, goal}))
        .step(starts, new int[] {1, 0}, List.of());

    assertThat(next).isNotNull();
    assertThat(next[1]).isEqualTo(goal);
    assertThat(next[0])
        .as("the follower must not enter the leader's just-vacated cell")
        .isEqualTo(followerStart);
  }

  @Test
  void constraintCannotForceAnAgentToFollowIntoAnOccupiedCell() {
    Roadmap map = GridFixture.grid(3, 1);
    int followerStart = GridFixture.id(map, 0, 0);
    int leaderStart = GridFixture.id(map, 0, 1);
    int goal = GridFixture.id(map, 0, 2);
    int[] starts = {followerStart, leaderStart};

    int[] next = new PibtEngine(map, 2, 11, DistTable.weighted(map, new int[] {goal, goal}))
        .step(starts, new int[] {1, 0}, List.of(
            new PibtEngine.Constraint(1, goal),
            new PibtEngine.Constraint(0, leaderStart)));

    assertThat(next)
        .as("a LaCAM constraint must not bypass the same hard follower rule")
        .isNull();
  }

  @Test
  void hardFollowerRuleHoldsTheFourAgentRing() {
    Roadmap map = GridFixture.grid(2, 2);
    int[] cells = ringCells(map);

    int[] next = rotateRing(map, 1);

    assertThat(next).isNotNull();
    assertThat(next)
        .as("each desired destination is occupied at tick start, so following is forbidden")
        .isEqualTo(cells);
    assertThat(GridFixture.hasAdgCycle(oneStepPlan(cells, next))).isFalse();
  }

  @Test
  void hardFollowerRuleHoldsForEverySeed() {
    Roadmap map = GridFixture.grid(2, 2);
    int[] cells = ringCells(map);

    for (int seed = 0; seed < 32; seed++) {
      assertThat(rotateRing(map, seed)).isEqualTo(cells);
    }
  }

  @Test
  void holeLessRingIsUnsolvableBecauseTheFleetCannotFollowIntoOccupiedCells() {
    Roadmap map = GridFixture.grid(2, 2);
    int[] cells = ringCells(map);

    Lacam2Solver.Result result = new Lacam2Solver(map)
        .solve(new MapfInstance(cells, ringGoals(cells)), 5000, 0,
            Lacam2Solver.Objective.NONE, Lacam2Solver.DEFAULT_RESTART_RATE,
            Lacam2Solver.LOOP_LIMIT_UNBOUNDED, SolverParams.WINDOW_FULL,
            null, null, null, null, null, StaticBodies.NONE);

    assertThat(result.solved())
        .as("every solution to a hole-less ring is a forbidden following rotation")
        .isFalse();
  }

  @Test
  void convoyOnlyMovesItsHeadIntoTheEmptyCell() {
    Roadmap map = GridFixture.grid(4, 1);
    int[] starts = {
        GridFixture.id(map, 0, 0), GridFixture.id(map, 0, 1), GridFixture.id(map, 0, 2)
    };
    int goal = GridFixture.id(map, 0, 3);
    DistTable dist = DistTable.weighted(map, new int[] {goal, goal, goal});

    int[] next = new PibtEngine(map, 3, 5, dist).step(starts, new int[] {0, 1, 2}, List.of());

    assertThat(next).isNotNull();
    int[][] plan = oneStepPlan(starts, next);
    assertThat(GridFixture.findCollisions(plan)).isEmpty();
    assertThat(GridFixture.findIllegalMoves(map, plan)).isEmpty();
    assertThat(GridFixture.hasAdgCycle(plan)).isFalse();
    assertThat(next).containsExactly(starts[0], starts[1], goal);
  }

  @Test
  void prefersTheCheaperNeighbourEvenWhenItSitsFartherFromTheGoal() {
    Roadmap r = weightedForkMap();
    int start = r.idOf("S");
    int plainBranch = r.idOf("A");
    int cargoBranch = r.idOf("B");
    DistTable dist = DistTable.weighted(r, new int[] { r.idOf("G") });

    assertThat(dist.get(0, cargoBranch))
        .as("the cargo branch is nearer the goal, so ordering on dist alone would take it")
        .isLessThan(dist.get(0, plainBranch));
    assertThat(r.weightOf(plainBranch) + dist.get(0, plainBranch))
        .as("yet stepping into the cargo branch costs more than the whole plain branch")
        .isLessThan(r.weightOf(cargoBranch) + dist.get(0, cargoBranch));

    PibtEngine pibt = new PibtEngine(r, 1, 11, dist);
    int[] next = pibt.step(new int[] { start }, new int[] { 0 }, List.of());

    assertThat(next).isNotNull();
    assertThat(next[0]).isEqualTo(plainBranch);
  }

  private static Roadmap weightedForkMap() {
    List<Roadmap.Point> points = List.of(
        new Roadmap.Point("S", 0, 0, 0),
        new Roadmap.Point("A", 1, 1, 0),
        new Roadmap.Point("X", 2, 1, 0),
        new Roadmap.Point("Y", 3, 1, 0),
        new Roadmap.Point("B", 1, -1, 0),
        new Roadmap.Point("Z", 2, -1, 0),
        new Roadmap.Point("G", 4, 0, 0));
    List<Roadmap.Path> paths = List.of(
        new Roadmap.Path("S-A", "S", "A", 1, true),
        new Roadmap.Path("A-X", "A", "X", 1, true),
        new Roadmap.Path("X-Y", "X", "Y", 1, true),
        new Roadmap.Path("Y-G", "Y", "G", 1, true),
        new Roadmap.Path("S-B", "S", "B", 1, true),
        new Roadmap.Path("B-Z", "B", "Z", 1, true),
        new Roadmap.Path("Z-G", "Z", "G", 1, true));
    return new Roadmap(new Roadmap.Model(points, paths, Set.of("B")));
  }

  @Test
  void neverSwapsOnNarrowCorridor() {
    Roadmap r = GridFixture.grid(2, 1);
    int c0 = GridFixture.id(r, 0, 0);
    int c1 = GridFixture.id(r, 0, 1);
    int[] starts = { c0, c1 };
    int[] goals = { c1, c0 };
    PibtEngine pibt = new PibtEngine(r, 2, 3, DistTable.weighted(r, goals));
    int[][] paths = GridFixture.rollout(pibt, r, starts, goals, 16);

    // They cannot pass (no room), but must stay collision-free the whole time.
    assertThat(GridFixture.findCollisions(paths)).isEmpty();
    assertThat(GridFixture.findIllegalMoves(r, paths)).isEmpty();
  }

  private static int[] ringCells(Roadmap map) {
    return new int[] {
        GridFixture.id(map, 0, 0), GridFixture.id(map, 0, 1),
        GridFixture.id(map, 1, 1), GridFixture.id(map, 1, 0)
    };
  }

  private static int[] ringGoals(int[] cells) {
    return new int[] {cells[1], cells[2], cells[3], cells[0]};
  }

  private static int[] rotateRing(Roadmap map, int seed) {
    int[] cells = ringCells(map);
    return new PibtEngine(map, 4, seed, DistTable.weighted(map, ringGoals(cells)))
        .step(cells, new int[] {0, 1, 2, 3}, List.of());
  }

  private static int[][] oneStepPlan(int[] from, int[] next) {
    int[][] plan = new int[from.length][2];
    for (int agent = 0; agent < from.length; agent++) {
      plan[agent][0] = from[agent];
      plan[agent][1] = next[agent];
    }
    return plan;
  }
}
