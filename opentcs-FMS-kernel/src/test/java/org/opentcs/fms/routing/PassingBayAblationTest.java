package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class PassingBayAblationTest {

  private static final String MAP_FILE = "v7-vda5050-test.xml";
  private static final long DEADLINE_MS = 120_000L;
  private static final int LOOP_LIMIT = 8_000;
  private static final int PRODUCTION_WINDOW = 10;
  private static final int MAX_ROUNDS = 40;
  private static final double UPPER_MAINLINE = 1000;
  private static final double LOWER_MAINLINE = 0;
  private static final int FLEET_PER_DIRECTION = 5;

  private static Roadmap map;

  @BeforeAll
  static void loadMap() throws Exception {
    File file = RoadmapXmlLoader.findMap(MAP_FILE);
    assumeTrue(file != null, MAP_FILE + " not found; skipping");
    map = new Roadmap(RoadmapXmlLoader.load(file));
  }

  @Test
  void passingBayAtFullWindow() {
    List<Integer> lane = pointsOn(LOWER_MAINLINE);
    int agents = FLEET_PER_DIRECTION * 2;
    int[] starts = headOnStarts(lane);
    int[] goals = headOnGoals(lane);

    Lacam2Solver.Result result = solve(starts, goals, SolverParams.WINDOW_FULL);
    int[][] paths = transpose(result.solution(), agents);
    List<String> collisions = GridFixture.findCollisions(paths);
    int upper = ticksOn(paths, UPPER_MAINLINE);

    report("FULL", lane.size(), starts, result.solved(), result.reachedGoals(),
        collisions.size(), upper, result.solution().size(), 1,
        result.loopCount(), result.nodesGenerated(), result.runtimeMs());

    assertThat(result.solved()).as("FULL: solver returned a plan").isTrue();
    assertThat(collisions).as("FULL: plan is collision free").isEmpty();
    assertThat(result.reachedGoals()).as("FULL: ten vehicles head-on still resolve").isTrue();
    assertThat(upper).as("FULL: vehicles borrow the upper mainline as a passing bay").isPositive();
  }

  @Test
  void passingBayAtProductionWindow() {
    List<Integer> lane = pointsOn(LOWER_MAINLINE);
    int agents = FLEET_PER_DIRECTION * 2;
    int[] starts = headOnStarts(lane);
    int[] goals = headOnGoals(lane);

    int[] current = starts.clone();
    int upper = 0;
    int steps = 0;
    int rounds = 0;
    int collisions = 0;
    long loops = 0;
    long nodes = 0;
    long millis = 0;
    boolean stalled = false;

    while (rounds < MAX_ROUNDS && !Arrays.equals(current, goals)) {
      Lacam2Solver.Result result = solve(current, goals, PRODUCTION_WINDOW);
      rounds++;
      loops += result.loopCount();
      nodes += result.nodesGenerated();
      millis += result.runtimeMs();
      if (!result.solved()) {
        stalled = true;
        break;
      }
      int[][] paths = transpose(result.solution(), agents);
      collisions += GridFixture.findCollisions(paths).size();
      upper += ticksOn(paths, UPPER_MAINLINE);
      steps += result.solution().size() - 1;
      int[] advanced = result.solution().get(result.solution().size() - 1);
      if (Arrays.equals(advanced, current)) {
        stalled = true;
        break;
      }
      current = advanced.clone();
    }

    boolean arrived = Arrays.equals(current, goals);
    report("10", lane.size(), starts, !stalled, arrived, collisions, upper, steps, rounds,
        loops, nodes, millis);

    assertThat(stalled).as("window=10: receding horizon never stalled").isFalse();
    assertThat(collisions).as("window=10: every windowed plan is collision free").isZero();
    assertThat(arrived).as("window=10: head-on traffic eventually clears").isTrue();
    assertThat(upper)
        .as("window=10: vehicles borrow the upper mainline as a passing bay")
        .isPositive();
  }

  private static int[] headOnStarts(List<Integer> lane) {
    int[] starts = new int[FLEET_PER_DIRECTION * 2];
    for (int i = 0; i < FLEET_PER_DIRECTION; i++) {
      starts[i] = lane.get(8 + i);
      starts[FLEET_PER_DIRECTION + i] = lane.get(lane.size() - 8 - i);
    }
    return starts;
  }

  private static int[] headOnGoals(List<Integer> lane) {
    int[] goals = new int[FLEET_PER_DIRECTION * 2];
    for (int i = 0; i < FLEET_PER_DIRECTION; i++) {
      goals[i] = lane.get(lane.size() - 9 - i);
      goals[FLEET_PER_DIRECTION + i] = lane.get(9 + i);
    }
    return goals;
  }

  private static Lacam2Solver.Result solve(int[] starts, int[] goals, int window) {
    return new Lacam2Solver(map).solve(
        new MapfInstance(starts, goals), DEADLINE_MS, 0,
        Lacam2Solver.Objective.SUM_OF_LOSS, Lacam2Solver.DEFAULT_RESTART_RATE,
        LOOP_LIMIT, window, null, null, null, null, null, StaticBodies.NONE);
  }

  private static void report(
      String window, int laneSize, int[] starts, boolean solved, boolean arrived,
      int collisions, int upperTicks, int steps, int rounds, long loops, long nodes,
      long millis) {
    System.out.printf(
        "[ABLATION] window=%s footprint=%s laneSize=%d gap=%d"
            + " solved=%s arrived=%s collisions=%d upperTicks=%d steps=%d rounds=%d"
            + " loops=%d nodes=%d ms=%d%n",
        window,
        System.getProperty("fms.pibt.footprint", "true"),
        laneSize, laneSize - 16, solved, arrived, collisions, upperTicks, steps, rounds,
        loops, nodes, millis);
  }

  private static List<Integer> pointsOn(double lane) {
    List<Integer> points = new ArrayList<>();
    for (int point = 0; point < map.size(); point++) {
      if (map.yOf(point) == lane) {
        points.add(point);
      }
    }
    points.sort((a, b) -> Double.compare(map.xOf(a), map.xOf(b)));
    return points;
  }

  private static int ticksOn(int[][] paths, double lane) {
    int ticks = 0;
    for (int[] path : paths) {
      for (int vertex : path) {
        ticks += map.yOf(vertex) == lane ? 1 : 0;
      }
    }
    return ticks;
  }

  private static int[][] transpose(List<int[]> plan, int agents) {
    int[][] paths = new int[agents][plan.size()];
    for (int t = 0; t < plan.size(); t++) {
      for (int a = 0; a < agents; a++) {
        paths[a][t] = plan.get(t)[a];
      }
    }
    return paths;
  }
}
