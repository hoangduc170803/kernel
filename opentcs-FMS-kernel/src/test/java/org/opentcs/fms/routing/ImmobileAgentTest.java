package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;

class ImmobileAgentTest {

  private static final String CORRIDOR_ENTRY = "1001";
  private static final String CORRIDOR_EXIT = "3090";

  private static final long DEADLINE_MS = 60_000L;
  private static final int LOOP_LIMIT = 48_000;

  private static Lacam2Solver.Result solveAlone(
      Roadmap roadmap, int start, int goal, int window) {
    return new Lacam2Solver(roadmap).solve(
        new MapfInstance(new int[] {start}, new int[] {goal}), DEADLINE_MS, 0,
        Lacam2Solver.Objective.SUM_OF_LOSS, Lacam2Solver.DEFAULT_RESTART_RATE,
        LOOP_LIMIT, window, null, null, null, null, null, StaticBodies.NONE);
  }

  private static int[] pathOfSoleAgent(Lacam2Solver.Result result) {
    List<int[]> configs = result.solution();
    int[] path = new int[configs.size()];
    for (int t = 0; t < configs.size(); t++) {
      path[t] = configs.get(t)[0];
    }
    return path;
  }


  @Test
  void blockingHidesTheVertexFromMovesButNotFromTheDistanceGradient() {
    Roadmap grid = GridFixture.grid(5, 5);
    int centre = GridFixture.id(grid, 2, 2);
    int west = GridFixture.id(grid, 2, 1);

    Roadmap masked = grid.blocking(Set.of(centre));

    assertThat(masked.neighborsOf(west)).doesNotContain(centre);
    assertThat(masked.candidates(west)).doesNotContain(centre);
    assertThat(grid.neighborsOf(west)).contains(centre);

    assertThat(masked.distanceTo(centre)).isEqualTo(grid.distanceTo(centre));
    assertThat(masked.distance(west, centre)).isEqualTo(1);
    assertThat(masked.size()).isEqualTo(grid.size());
  }

  @Test
  void blockingIsIdentityWhenNothingIsBlocked() {
    Roadmap grid = GridFixture.grid(4, 4);
    assertThat(grid.blocking(Set.of())).isSameAs(grid);
  }

  @Test
  void anAgentWhoseGoalIsBlockedWaitsAdjacentInsteadOfEnteringOrFreezingFarAway() {
    Roadmap grid = GridFixture.grid(5, 5);
    int centre = GridFixture.id(grid, 2, 2);
    int start = GridFixture.id(grid, 2, 0);

    Lacam2Solver.Result result = solveAlone(grid.blocking(Set.of(centre)), start, centre, 4);

    int[] path = pathOfSoleAgent(result);
    assertThat(path).doesNotContain(centre);
    assertThat(result.reachedGoals())
        .as("the goal is walled off, so no configuration in the window stands on it")
        .isFalse();
    assertThat(path[path.length - 1])
        .as("waiting is a self-loop on the configuration graph, so LaCAM cannot deepen a node by"
            + " standing still: every windowed terminal is a walk, and none of them parks the"
            + " agent next to the blocked goal")
        .isNotEqualTo(GridFixture.id(grid, 2, 1));
  }

  @Test
  void aBlockedVertexIsNeverEnteredEvenWhenItIsOnTheOnlyRoute() throws Exception {
    Roadmap roadmap = realMap();
    assumeTrue(roadmap != null, "v7-vda5050-test.xml not found; skipping");

    List<String> corridor = gradientRoute(roadmap, CORRIDOR_ENTRY, CORRIDOR_EXIT);
    String barrier = corridor.get(corridor.size() / 2);
    int barrierPoint = roadmap.idOf(barrier);
    int entry = roadmap.idOf(CORRIDOR_ENTRY);
    int exit = roadmap.idOf(CORRIDOR_EXIT);

    Lacam2Solver.Result result = solveAlone(roadmap.blocking(Set.of(barrierPoint)), entry, exit, 10);

    int[] path = pathOfSoleAgent(result);
    int[] toExit = roadmap.distanceTo(exit);

    assertThat(path).doesNotContain(barrierPoint);
    assertThat(toExit[path[path.length - 1]])
        .as("the agent must still approach the goal, not sit at the start")
        .isLessThan(toExit[entry]);
  }

  private static Roadmap realMap() throws Exception {
    File map = RoadmapXmlLoader.findMap("v7-vda5050-test.xml");
    return map == null ? null : new Roadmap(RoadmapXmlLoader.load(map));
  }

  private static List<String> gradientRoute(Roadmap roadmap, String from, String to) {
    int[] dist = roadmap.distanceTo(roadmap.idOf(to));
    List<String> route = new ArrayList<>();
    int current = roadmap.idOf(from);
    route.add(roadmap.nameOf(current));
    while (dist[current] > 0) {
      int best = current;
      for (int u : roadmap.neighborsOf(current)) {
        if (dist[u] < dist[best]) {
          best = u;
        }
      }
      current = best;
      route.add(roadmap.nameOf(current));
    }
    return route;
  }

}
