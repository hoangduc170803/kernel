package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ParallelLaneSwapTest {

  private static final String MAP_FILE = "v7-Xuonghientai_them.xml";
  private static final int WINDOW = 10;
  private static final long DEADLINE_MS = 1_000L;
  private static final int SEED_COUNT = 50;
  private static final Set<String> UPPER_LANE = Set.of(
      "0495", "0496", "0497", "0498", "0499", "0501", "0586", "0588",
      "0589", "0590", "0591", "0592", "0593", "0594");
  private static final Set<String> LOWER_LANE = Set.of(
      "0535", "0536", "0538", "0539", "0540", "0541", "0542", "0543",
      "0544", "0545", "0546", "0548", "0550", "0551", "0552", "0553");

  private static Roadmap map;

  @BeforeAll
  static void loadTheIncidentMap() throws Exception {
    File file = RoadmapXmlLoader.findMap(MAP_FILE);
    assumeTrue(file != null, MAP_FILE + " not found; skipping");
    map = new Roadmap(RoadmapXmlLoader.load(file));
  }

  @BeforeEach
  void armTheFootprint() {
    System.setProperty("fms.pibt.footprint", "true");
  }

  @AfterEach
  void restoreDefaults() {
    System.clearProperty("fms.pibt.footprint");
  }

  @Test
  void twoLaneVehiclesKeepTheirLanesWhenTheGuidePlanningOrderFlips() {
    assertIncidentMapGeometry();

    SweepSummary solveAGuides = new SweepSummary("A", "guide");
    SweepSummary solveAPlans = new SweepSummary("A", "plan");
    SweepSummary solveBGuides = new SweepSummary("B", "guide");
    SweepSummary solveBPlans = new SweepSummary("B", "plan");

    for (int seed = 0; seed < SEED_COUNT; seed++) {
      Lacam2Solver.Result solveA = solve(solveAInputs(seed));
      recordGuideOutcome(solveAGuides, solveA, 0, 1);
      recordPlanOutcome(solveAPlans, solveA, 0, 1);

      Lacam2Solver.Result solveB = solve(solveBInputs(seed, solveA));
      recordGuideOutcome(solveBGuides, solveB, 0, 1);
      recordPlanOutcome(solveBPlans, solveB, 0, 1);
    }

    for (SweepSummary sweep : List.of(solveAGuides, solveAPlans, solveBGuides, solveBPlans)) {
      assertThat(sweep.vehicle1LaneSwitches())
          .as("Vehicle-0001 lane switches in solve %s %s", sweep.scenario(), sweep.phase())
          .isZero();
      assertThat(sweep.vehicle2LaneSwitches())
          .as("Vehicle-0002 lane switches in solve %s %s", sweep.scenario(), sweep.phase())
          .isZero();
      assertThat(sweep.unsolvedSeeds())
          .as("unsolved seeds in solve %s %s", sweep.scenario(), sweep.phase())
          .isZero();
    }
  }

  private static void assertIncidentMapGeometry() {
    String[] referencedPoints = {
        "0495", "0496", "0497", "0498", "0499", "0501", "0586", "0588", "0589",
        "0590", "0591", "0592", "0593", "0594", "0535", "0536", "0538", "0539",
        "0540", "0541", "0542", "0543", "0544", "0545", "0546", "0548", "0550",
        "0551", "0552", "0553", "0065", "0354", "0135", "0134", "0144", "0154",
        "0162", "0153", "0376", "0143", "0156", "0040", "0378", "0312"};
    for (String point : referencedPoints) {
      assertThat(map.has(point)).as("%s missing from %s", point, MAP_FILE).isTrue();
    }

    assertLaneCoordinates(
        new String[] {"0495", "0496", "0497", "0498", "0499", "0501", "0586", "0588",
            "0589", "0590", "0591", "0592", "0593", "0594"},
        new double[] {19700, 18750, 17800, 16900, 15900, 14950, 14000, 13000,
            12050, 11150, 10200, 9250, 8300, 7350},
        -3250);
    assertLaneCoordinates(
        new String[] {"0535", "0536", "0538", "0539", "0540", "0541", "0542", "0543",
            "0544", "0545", "0546", "0548"},
        new double[] {21600, 20650, 19700, 18750, 17800, 16900, 15900, 14950,
            14000, 13000, 12050, 11150},
        -4050);

    assertThat(map.neighborsOf(id("0498"))).contains(id("0065"));
    assertThat(map.neighborsOf(id("0553"))).contains(id("0135"));
    assertThat(map.neighborsOf(id("0552"))).contains(id("0134"));
  }

  private static void assertLaneCoordinates(String[] points, double[] xCoordinates, double y) {
    for (int index = 0; index < points.length; index++) {
      assertThat(map.xOf(id(points[index]))).as("%s x-coordinate", points[index])
          .isEqualTo(xCoordinates[index]);
      assertThat(map.yOf(id(points[index]))).as("%s y-coordinate", points[index])
          .isEqualTo(y);
    }
  }

  private static SolveInputs solveAInputs(int seed) {
    String[] agents = {
        "Vehicle-0001", "Vehicle-0002", "Vehicle-0003", "Vehicle-0006", "Vehicle-0008"};
    String[][] assumedCommittedPrefixesWithOnlyVehicle1DirectlyLogged = {
        {"0593", "0592", "0591"},
        {"0535", "0536", "0538"},
        {"0133", "0143", "0153"},
        {"0542", "0543", "0544"},
        {"0155", "0156", "0146"}};
    int[][] assumedSeedPathsFromTheLastAvailableCapture = {
        path("0552", "0593", "0593", "0592", "0591", "0590", "0589", "0588", "0586",
            "0501", "0499", "0498", "0065", "0064", "0332", "0354"),
        path("0387", "0535", "0536", "0538", "0539", "0540", "0541", "0542", "0543",
            "0544", "0545", "0546", "0548", "0550", "0551", "0552", "0553", "0135"),
        path("0551", "0133", "0143", "0153"),
        path("0541", "0542", "0543", "0544", "0545", "0546", "0548", "0550", "0551",
            "0552", "0134", "0144", "0154"),
        null};
    int[] startPositions = ids("0593", "0535", "0133", "0542", "0155");
    int[] startMotions = startMotionsFromSeedPaths(assumedSeedPathsFromTheLastAvailableCapture,
        startPositions);
    AgentPriority[] assumedPrioritiesFromTheLastAvailableCapture = {
        new AgentPriority(0, 0, 35, 0, 0.0f, 5),
        new AgentPriority(0, 0, 85, 0, 0.0f, 3),
        new AgentPriority(0, 0, 40, 0, 0.0f, 4),
        new AgentPriority(0, 0, 86, 0, 0.0f, 2),
        new AgentPriority(0, 0, 86, -1, 0.0f, 0)};

    return new SolveInputs(
        "A",
        agents,
        new String[] {"0593", "0535", "0133", "0542", "0155"},
        new String[] {"0354", "0135", "0153", "0154", "0376"},
        new int[] {
            Roadmap.DIR_POS_Y, Roadmap.DIR_NEG_X, Roadmap.DIR_NEG_Y,
            Roadmap.DIR_NEG_X, Roadmap.DIR_NEG_Y},
        new boolean[] {false, true, true, true, false},
        new String[] {"0378", "0040", "0312"},
        new String[] {"0378", "0040", "0312"},
        new int[] {Roadmap.DIR_NEG_Y, Roadmap.DIR_NEG_Y, Roadmap.DIR_NEG_Y},
        new boolean[] {false, true, true},
        assumedCommittedPrefixesWithOnlyVehicle1DirectlyLogged,
        assumedPrioritiesFromTheLastAvailableCapture,
        assumedSeedPathsFromTheLastAvailableCapture,
        startMotions,
        seed);
  }

  private static SolveInputs solveBInputs(int seed, Lacam2Solver.Result solveA) {
    String[] agents = {
        "Vehicle-0001", "Vehicle-0002", "Vehicle-0003", "Vehicle-0005", "Vehicle-0006",
        "Vehicle-0008"};
    String[][] committedPrefixesWithVehicle1Vehicle2AndVehicle8DirectlyLogged = {
        {"0593", "0592", "0591"},
        {"0536", "0538", "0539"},
        {"0143", "0153"},
        {"0040"},
        {"0543", "0544", "0545"},
        {"0155", "0156", "0146"}};
    String[] starts = {"0593", "0536", "0143", "0040", "0543", "0155"};
    int[] startPositions = ids(starts);
    int[][] seedPathsFromSolveA = seedPathsFromPriorSolution(
        solveA, new String[] {
            "Vehicle-0001", "Vehicle-0002", "Vehicle-0003", "Vehicle-0006", "Vehicle-0008"},
        agents, new String[] {"0354", "0135", "0153", "0162", "0154", "0376"},
        new String[] {"0354", "0135", "0153", null, "0154", "0376"}, startPositions);

    return new SolveInputs(
        "B",
        agents,
        starts,
        new String[] {"0354", "0135", "0153", "0162", "0154", "0376"},
        new int[] {
            Roadmap.DIR_POS_Y, Roadmap.DIR_NEG_X, Roadmap.DIR_NEG_Y,
            Roadmap.DIR_NEG_Y, Roadmap.DIR_NEG_X, Roadmap.DIR_NEG_Y},
        new boolean[] {false, true, true, true, true, false},
        new String[] {"0378", "0312"},
        new String[] {"0378", "0312"},
        new int[] {Roadmap.DIR_NEG_Y, Roadmap.DIR_NEG_Y},
        new boolean[] {false, true},
        committedPrefixesWithVehicle1Vehicle2AndVehicle8DirectlyLogged,
        rootPrioritiesCarriedFromSolveA(solveA),
        seedPathsFromSolveA,
        startMotionsFromSeedPaths(seedPathsFromSolveA, startPositions),
        seed);
  }

  private static AgentPriority[] rootPrioritiesCarriedFromSolveA(
      Lacam2Solver.Result solveA) {
    AgentPriority[] oldRootPriorities = solveAInputs(0).rootPriorities();
    AgentPriority[] executedPriorities = solveA.executedPriorities();
    if (executedPriorities == null || executedPriorities.length != oldRootPriorities.length) {
      executedPriorities = oldRootPriorities;
    }
    int[] rememberedRanks = orderRanksOf(executedPriorities);
    return new AgentPriority[] {
        carriedPriority(oldRootPriorities[0], executedPriorities[0], rememberedRanks[0]),
        carriedPriority(oldRootPriorities[1], executedPriorities[1], rememberedRanks[1]),
        carriedPriority(oldRootPriorities[2], executedPriorities[2], rememberedRanks[2]),
        AgentPriority.NEWCOMER,
        carriedPriority(oldRootPriorities[3], executedPriorities[3], rememberedRanks[3]),
        carriedPriority(oldRootPriorities[4], executedPriorities[4], rememberedRanks[4])};
  }

  private static AgentPriority carriedPriority(
      AgentPriority previousRoot, AgentPriority previousExecution, int previousOrderRank) {
    return new AgentPriority(
        previousExecution.retreating(),
        previousExecution.inGoalLane(),
        previousRoot.aged() + 1,
        previousExecution.settled(),
        previousExecution.tie(),
        previousOrderRank);
  }

  private static int[] orderRanksOf(AgentPriority[] priorities) {
    Integer[] order = new Integer[priorities.length];
    for (int index = 0; index < order.length; index++) {
      order[index] = index;
    }
    Arrays.sort(order, (left, right) -> priorities[right].compareTo(priorities[left]));
    int[] ranks = new int[priorities.length];
    for (int rank = 0; rank < order.length; rank++) {
      ranks[order[rank]] = rank;
    }
    return ranks;
  }

  private static int[][] seedPathsFromPriorSolution(
      Lacam2Solver.Result prior,
      String[] priorAgents,
      String[] currentAgents,
      String[] currentGoals,
      String[] priorGoals,
      int[] currentStarts) {
    int[][] seedPaths = new int[currentAgents.length][];
    for (int currentAgent = 0; currentAgent < currentAgents.length; currentAgent++) {
      int priorAgent = Arrays.asList(priorAgents).indexOf(currentAgents[currentAgent]);
      if (priorAgent < 0 || !currentGoals[currentAgent].equals(priorGoals[currentAgent])) {
        continue;
      }
      int[] previousPath = pathForAgent(prior.solution(), priorAgent);
      seedPaths[currentAgent] = pathFromWhereTheVehicleActuallyIs(
          previousPath, currentStarts[currentAgent]);
    }
    return seedPaths;
  }

  private static int[] pathForAgent(List<int[]> solution, int agent) {
    int[] path = new int[solution.size()];
    for (int time = 0; time < solution.size(); time++) {
      path[time] = solution.get(time)[agent];
    }
    return path;
  }

  private static int[] pathFromWhereTheVehicleActuallyIs(int[] plan, int start) {
    for (int time = 2; time < plan.length; time++) {
      if (plan[time] == start) {
        return Arrays.copyOfRange(plan, time - 1, plan.length);
      }
    }
    return plan;
  }

  private static int[] startMotionsFromSeedPaths(int[][] seedPaths, int[] starts) {
    int[] startMotions = new int[starts.length];
    Arrays.fill(startMotions, Roadmap.DIR_NONE);
    for (int agent = 0; agent < starts.length; agent++) {
      int[] seedPath = seedPaths[agent];
      if (seedPath != null && seedPath.length > 1 && seedPath[1] == starts[agent]) {
        startMotions[agent] = map.motionAfter(seedPath[0], seedPath[1]);
      }
    }
    return startMotions;
  }

  private static Lacam2Solver.Result solve(SolveInputs inputs) {
    int[] starts = ids(inputs.starts());
    int[] goals = ids(inputs.goals());
    int[] blockedCells = ids(inputs.blockedCells());
    Set<Integer> blockedVertices = Arrays.stream(blockedCells).boxed()
        .collect(java.util.stream.Collectors.toUnmodifiableSet());
    Roadmap planningRoadmap = map.blocking(blockedVertices);
    StaticBodies staticBodies = new StaticBodies(
        ids(inputs.staticCells()), inputs.staticNoses(), inputs.staticLoaded());
    int[][] committedPrefixes = ids(inputs.committedPrefixes());
    Lacam2Solver.Result result = new Lacam2Solver(planningRoadmap).solve(
        new MapfInstance(starts, goals),
        DEADLINE_MS,
        inputs.seed(),
        Lacam2Solver.Objective.NONE,
        Lacam2Solver.DEFAULT_RESTART_RATE,
        Lacam2Solver.LOOP_LIMIT_UNBOUNDED,
        WINDOW,
        inputs.rootPriorities(),
        inputs.startNoses(),
        inputs.loaded(),
        inputs.seedGuidePaths(),
        committedPrefixes,
        staticBodies,
        inputs.startMotions());
    assertThat(result.solved()).as("solve %s seed %d", inputs.scenario(), inputs.seed()).isTrue();
    return result;
  }

  private static void recordGuideOutcome(
      SweepSummary summary, Lacam2Solver.Result result, int vehicle1, int vehicle2) {
    int[][] guidePaths = result.guidePaths();
    List<String> vehicle1Path = guidePaths == null ? List.of() : namesOf(guidePaths[vehicle1]);
    List<String> vehicle2Path = guidePaths == null ? List.of() : namesOf(guidePaths[vehicle2]);
    summary.record(
        crossesInto(vehicle1Path, LOWER_LANE), crossesInto(vehicle2Path, UPPER_LANE),
        result.solved());
  }

  private static void recordPlanOutcome(
      SweepSummary summary, Lacam2Solver.Result result, int vehicle1, int vehicle2) {
    List<String> vehicle1Path = namesOf(pathForAgent(result.solution(), vehicle1));
    List<String> vehicle2Path = namesOf(pathForAgent(result.solution(), vehicle2));
    summary.record(
        crossesInto(vehicle1Path, LOWER_LANE), crossesInto(vehicle2Path, UPPER_LANE),
        result.solved());
  }

  private static boolean crossesInto(List<String> path, Set<String> lane) {
    return path.stream().anyMatch(lane::contains);
  }

  private static List<String> namesOf(int[] path) {
    List<String> names = new ArrayList<>();
    if (path == null) {
      return names;
    }
    for (int vertex : path) {
      if (vertex >= 0) {
        names.add(map.nameOf(vertex));
      }
    }
    return names;
  }

  private static int[] path(String... names) {
    return ids(names);
  }

  private static int[] ids(String... names) {
    int[] result = new int[names.length];
    for (int index = 0; index < names.length; index++) {
      result[index] = id(names[index]);
    }
    return result;
  }

  private static int[][] ids(String[][] paths) {
    int[][] result = new int[paths.length][];
    for (int index = 0; index < paths.length; index++) {
      result[index] = paths[index] == null ? null : ids(paths[index]);
    }
    return result;
  }

  private static int id(String point) {
    return map.idOf(point);
  }

  private record SolveInputs(
      String scenario,
      String[] agents,
      String[] starts,
      String[] goals,
      int[] startNoses,
      boolean[] loaded,
      String[] blockedCells,
      String[] staticCells,
      int[] staticNoses,
      boolean[] staticLoaded,
      String[][] committedPrefixes,
      AgentPriority[] rootPriorities,
      int[][] seedGuidePaths,
      int[] startMotions,
      int seed) {
  }

  private static final class SweepSummary {

    private final String scenario;
    private final String phase;
    private int vehicle1LaneSwitches;
    private int vehicle2LaneSwitches;
    private int unsolvedSeeds;

    private SweepSummary(String scenario, String phase) {
      this.scenario = scenario;
      this.phase = phase;
    }

    private void record(boolean vehicle1Switches, boolean vehicle2Switches, boolean solved) {
      if (vehicle1Switches) {
        vehicle1LaneSwitches++;
      }
      if (vehicle2Switches) {
        vehicle2LaneSwitches++;
      }
      if (!solved) {
        unsolvedSeeds++;
      }
    }

    private int vehicle1LaneSwitches() {
      return vehicle1LaneSwitches;
    }

    private int vehicle2LaneSwitches() {
      return vehicle2LaneSwitches;
    }

    private int unsolvedSeeds() {
      return unsolvedSeeds;
    }

    private String scenario() {
      return scenario;
    }

    private String phase() {
      return phase;
    }
  }
}
