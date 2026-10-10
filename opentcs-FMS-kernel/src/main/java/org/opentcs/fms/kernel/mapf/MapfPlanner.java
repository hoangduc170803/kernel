package org.opentcs.fms.kernel.mapf;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import org.opentcs.fms.routing.AgentPriority;
import org.opentcs.fms.routing.Lacam2Solver;
import org.opentcs.fms.routing.LegalMoves;
import org.opentcs.fms.routing.MapfInstance;
import org.opentcs.fms.routing.PathUtils;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.SolverParams;
import org.opentcs.fms.routing.StaticBodies;
import org.opentcs.fms.routing.pibt.Footprint;

public final class MapfPlanner {

  private static final org.slf4j.Logger logger = org.slf4j.LoggerFactory.getLogger(MapfPlanner.class);

  private static final int LACAM2_LOOP_LIMIT = Integer.getInteger(
      "fms.mapf.lacam2LoopLimit", Lacam2Solver.LOOP_LIMIT_UNBOUNDED);

  private static final String LACAM2_LOOP_LIMIT_LABEL = LACAM2_LOOP_LIMIT == Lacam2Solver.LOOP_LIMIT_UNBOUNDED
      ? "unbounded"
      : String.valueOf(LACAM2_LOOP_LIMIT);

  static final int NO_VERTEX = -1;

  private final Roadmap roadmap;
  private final String modelName;
  private final SolverInputTransformer solverInputTransformer = new SolverInputTransformer();
  private final Map<String, AgentPriority> lastPriorities = new LinkedHashMap<>();
  private final Map<String, Integer> lastOrderRanks = new LinkedHashMap<>();
  private final AgentInDropOffColumn inDropOffColumn = new AgentInDropOffColumn();
  private String lastAgedLine;
  private Footprint goalGeometry;
  private final Map<String, int[]> lastPlanPaths = new LinkedHashMap<>();
  private final Map<String, Integer> lastPlanGoals = new LinkedHashMap<>();

  private static final int MOTION_SEED_LOG_EVERY_SOLVES = Math.max(
      1, Integer.getInteger("fms.mapf.motionSeedLogEvery", 100));

  private long motionSeededFromLastPlan;
  private long motionFellBackToStopped;
  private int solvesSinceMotionSeedLog;
  private final Map<String, Integer> goalsOfPlansDroppedWhenAVehicleLeftTheSolve = new LinkedHashMap<>();
  private final WarmStartMemoryLogger warmStartMemoryLogger = new WarmStartMemoryLogger(
      lastPlanGoals, goalsOfPlansDroppedWhenAVehicleLeftTheSolve);
  private final GoalClaimer goalClaimer;
  private final RouteShapeLogger routeShapeLogger;

  public MapfPlanner(Roadmap roadmap) {
    this(roadmap, "unknown");
  }

  public MapfPlanner(Roadmap roadmap, String modelName) {
    this.roadmap = roadmap;
    this.modelName = modelName;
    this.goalClaimer = new GoalClaimer(roadmap);
    this.routeShapeLogger = new RouteShapeLogger(roadmap);
  }

  public record PlanResult(
      Map<String, List<String>> routes,
      Map<String, List<String>> spaceTimePaths,
      Map<String, String> plannedGoals,
      int coordinatedTicks,
      boolean executable,
      Map<String, List<String>> solverCommittedPrefixes) {
  }

  public Map<String, List<String>> planRoutes(
      Map<String, String> positions, Map<String, String> goals, SolverParams params,
      ClaimPolicy claimPolicy) {
    return planRoutes(positions, goals, params, claimPolicy, List.of());
  }

  public Map<String, List<String>> planRoutes(
      Map<String, String> positions, Map<String, String> goals, SolverParams params,
      ClaimPolicy claimPolicy, List<Footprint.Obstacle> obstacles) {
    return plan(
        positions, goals, params, Set.of(), claimPolicy, BodyState.UNKNOWN,
        Map.of(), obstacles).routes();
  }

  public PlanResult plan(
      Map<String, String> positions, Map<String, String> goals, SolverParams params,
      Set<String> immobileVehicles,
      ClaimPolicy claimPolicy, BodyState bodyState,
      Map<String, List<String>> committedPrefixes) {
    return plan(
        positions, goals, params, immobileVehicles, claimPolicy, bodyState, committedPrefixes,
        List.of());
  }

  public PlanResult plan(
      Map<String, String> positions, Map<String, String> goals, SolverParams params,
      Set<String> immobileVehicles,
      ClaimPolicy claimPolicy, BodyState bodyState,
      Map<String, List<String>> committedPrefixes, List<Footprint.Obstacle> obstacles) {
    Map<String, List<String>> prefixes = Map.copyOf(committedPrefixes);
    List<String> agents = new ArrayList<>();
    Map<String, String> heldPoints = new LinkedHashMap<>();
    Map<String, String> parkedWithoutAnOrder = new LinkedHashMap<>();
    Set<Integer> occupiedByImmobile = new HashSet<>();
    for (Map.Entry<String, String> e : positions.entrySet()) {
      String pos = e.getValue();
      if (pos == null || !roadmap.has(pos)) {
        continue;
      }
      String goal = goals.get(e.getKey());
      boolean alreadyAtGoal = goal == null || !roadmap.has(goal) || goal.equals(pos);
      if (immobileVehicles.contains(e.getKey()) || alreadyAtGoal) {
        heldPoints.put(e.getKey(), pos);
        if (goal == null && !immobileVehicles.contains(e.getKey())) {
          parkedWithoutAnOrder.put(e.getKey(), pos);
        }
        occupiedByImmobile.add(roadmap.idOf(pos));
        continue;
      }
      agents.add(e.getKey());
    }
    Collections.sort(agents);
    resetPrioritiesOfVehiclesAtGoal(positions, claimPolicy);

    int n = agents.size();
    Roadmap planningRoadmap = occupiedByImmobile.isEmpty() ? roadmap : roadmap.blocking(occupiedByImmobile);
    int[] starts = new int[n];
    int[][] committedPrefixVertices = committedPrefixVertices(agents, prefixes);
    for (int a = 0; a < n; a++) {
      starts[a] = roadmap.idOf(positions.get(agents.get(a)));
    }
    int[] goalVertices = goalClaimer.claim(planningRoadmap, agents, goals, starts).goalVertices();
    int[] retreatGoals = solverInputTransformer
        .retreatGoals(agents, claimPolicy, planningRoadmap);
    LegalMoves moves = new LegalMoves(planningRoadmap, retreatGoals);

    Map<String, List<String>> routes = new LinkedHashMap<>();
    Map<String, List<String>> spaceTime = new LinkedHashMap<>();
    Map<String, String> plannedGoals = new LinkedHashMap<>();
    for (Map.Entry<String, String> held : heldPoints.entrySet()) {
      routes.put(held.getKey(), List.of(held.getValue()));
      spaceTime.put(held.getKey(), List.of(held.getValue()));
      plannedGoals.put(held.getKey(), held.getValue());
    }
    for (int a = 0; a < n; a++) {
      plannedGoals.put(agents.get(a), roadmap.nameOf(goalVertices[a]));
    }
    boolean executable = true;
    int coordinatedTicks = 1;
    if (n > 0) {
      if (!occupiedByImmobile.isEmpty()) {
        logger.debug(
            "[MAPF mask] {} vehicle(s) dropped from the agent set; their points are unreachable"
                + " for everyone this solve, but stay in the distance table so goals on/behind"
                + " them keep a gradient: {}",
            heldPoints.size(), heldPoints);
      }
      if (!parkedWithoutAnOrder.isEmpty()) {
        logger.debug(
            "[MAPF mask] {} of them hold a cell with no order at all: {}. Nothing can ask these"
                + " to move, so their bodies are obstacles for the whole solve and every cell"
                + " within reach of one is unusable until dispatch gives them somewhere to go.",
            parkedWithoutAnOrder.size(), parkedWithoutAnOrder);
      }
      int[][] dist = new int[n][];
      AgentPriority[] rootPriorities = new AgentPriority[n];
      int[] startAxis = new int[n];
      int[] startMotion = new int[n];
      boolean[] loaded = new boolean[n];
      int[][] seedGuidePaths = new int[n][];
      for (int a = 0; a < n; a++) {
        String vehicle = agents.get(a);
        dist[a] = planningRoadmap.distanceTo(goalVertices[a]);
        rootPriorities[a] = seededPriority(
            vehicle, claimPolicy, planningRoadmap, starts[a], goalVertices[a]);
        startAxis[a] = bodyState.axisOf(vehicle);
        loaded[a] = bodyState.loaded(vehicle);
        warmStartMemoryLogger.record(vehicle, goalVertices[a]);
        seedGuidePaths[a] = lastPlanGoals.getOrDefault(vehicle, NO_VERTEX) == goalVertices[a]
            ? planFromWhereTheVehicleActuallyIs(lastPlanPaths.get(vehicle), starts[a])
            : null;
        startMotion[a] = motionEnteringTheCurrentCell(
            planningRoadmap, seedGuidePaths[a], starts[a]);
      }
      warmStartMemoryLogger.logAndReset(n);
      reportGoalsThatCannotCoexist(agents, goalVertices, loaded);
      countAndLogMotionSeeds(startMotion);
      StaticBodies statics = staticBodiesOf(roadmap, heldPoints, bodyState)
          .withObstacles(obstacles);
      List<String> staticVehicleNames = new ArrayList<>(heldPoints.keySet());
      Set<String> blockedVertices = new TreeSet<>();
      for (int vertex : occupiedByImmobile) {
        blockedVertices.add(roadmap.nameOf(vertex));
      }
      FleetPlan solvedFleet = solveFleet(
          planningRoadmap, new MapfInstance(starts, goalVertices), agents, blockedVertices,
          staticVehicleNames, statics, params,
          rootPriorities, startAxis, loaded, seedGuidePaths, committedPrefixVertices,
          startMotion, moves);
      logger.debug(
          "[MAPF legal-moves] filtered={} forcingStay={}",
          moves.filteredCandidates(), moves.forcingStay());
      FleetPlan fleet = finalizeFleetPlan(
          planningRoadmap, solvedFleet, agents, goalVertices, dist, claimPolicy,
          occupiedByImmobile.size(), params.window());
      executable = fleet.executable();
      coordinatedTicks = fleet.coordinatedTicks();
      rememberPriorities(agents, fleet.priorities(), claimPolicy);
      rememberOrder(agents, fleet.priorities(), executable);
      rememberPlanPaths(agents, fleet.paths(), goalVertices, executable);

      for (int a = 0; a < n; a++) {
        int[] raw = fleet.paths()[a];

        List<String> timed = new ArrayList<>(raw.length);
        for (int v : raw) {
          timed.add(roadmap.nameOf(v));
        }
        spaceTime.put(agents.get(a), timed);

        int[] simple = PathUtils.compress(raw);
        routeShapeLogger.logAnomalies(
            agents.get(a), raw, simple, goalVertices[a],
            fleet.guidePaths() == null ? null : fleet.guidePaths()[a]);
        List<String> names = new ArrayList<>(simple.length);
        for (int v : simple) {
          names.add(roadmap.nameOf(v));
        }
        routes.put(agents.get(a), names);
        logger.debug("[MAPF plan-dump] {} route({}): {}", agents.get(a), names.size(), names);
      }
    }
    logAged();
    return new PlanResult(
        routes, spaceTime, plannedGoals, coordinatedTicks, executable,
        namesOfCommittedPrefixes(agents, committedPrefixVertices));
  }

  private record FleetPlan(
      int[][] paths, boolean executable, AgentPriority[] priorities, int[][] guidePaths,
      int coordinatedTicks, Lacam2Solver.Result solverResult) {

    FleetPlan withPaths(int[][] adjustedPaths) {
      return new FleetPlan(
          adjustedPaths, executable, priorities, guidePaths, coordinatedTicks, solverResult);
    }
  }

  private AgentPriority seededPriority(
      String vehicle, ClaimPolicy claimPolicy, Roadmap map, int start, int goal) {
    AgentPriority remembered = lastPriorities.getOrDefault(vehicle, AgentPriority.NEWCOMER);
    int body = inDropOffColumn.bodyVertexOf(vehicle, claimPolicy, map, start);
    return new AgentPriority(
        claimPolicy.isRetreating(vehicle) ? 1 : 0,
        inDropOffColumn.test(vehicle, claimPolicy, map, body, goal) ? 1 : 0,
        remembered.aged(),
        remembered.settled(),
        0f,
        lastOrderRanks.getOrDefault(vehicle, Integer.MAX_VALUE));
  }

  private void logAged() {
    StringBuilder out = new StringBuilder();
    lastPriorities.forEach(
        (vehicle, priority) -> out
            .append(out.length() == 0 ? "" : " ")
            .append(vehicle).append('=').append(priority.aged()));
    String line = out.toString();
    if (line.equals(lastAgedLine)) {
      return;
    }
    lastAgedLine = line;
    logger.debug("[MAPF aged] solves since each vehicle last reached a real goal: {}", line);
  }

  private void resetPrioritiesOfVehiclesAtGoal(
      Map<String, String> positions, ClaimPolicy claimPolicy) {
    for (String vehicle : positions.keySet()) {
      if (claimPolicy.isAtGoal(vehicle)) {
        lastPriorities.put(vehicle, AgentPriority.NEWCOMER);
      }
    }
  }

  private void rememberPriorities(
      List<String> agents, AgentPriority[] executed, ClaimPolicy claimPolicy) {
    if (executed == null || executed.length != agents.size()) {
      return;
    }
    for (int a = 0; a < agents.size(); a++) {
      String vehicle = agents.get(a);
      if (claimPolicy.isAtGoal(vehicle)) {
        continue;
      }
      AgentPriority solved = executed[a];
      int carried = lastPriorities.getOrDefault(vehicle, AgentPriority.NEWCOMER).aged() + 1;
      lastPriorities.put(vehicle, new AgentPriority(
          solved.retreating(), solved.inGoalLane(), carried, solved.settled(), solved.tie()));
    }
  }

  private void rememberOrder(
      List<String> agents, AgentPriority[] executed, boolean executable) {
    if (!executable || executed == null || executed.length != agents.size()) {
      return;
    }
    Integer[] order = new Integer[agents.size()];
    for (int agent = 0; agent < agents.size(); agent++) {
      order[agent] = agent;
    }
    Arrays.sort(order, (left, right) -> executed[right].compareTo(executed[left]));
    lastOrderRanks.clear();
    for (int rank = 0; rank < order.length; rank++) {
      lastOrderRanks.put(agents.get(order[rank]), rank);
    }
  }

  static int motionEnteringTheCurrentCell(Roadmap roadmap, int[] carriedPlan, int start) {
    if (carriedPlan == null || carriedPlan.length < 2 || carriedPlan[1] != start) {
      return Roadmap.DIR_NONE;
    }
    return roadmap.motionAfter(carriedPlan[0], carriedPlan[1]);
  }

  private void countAndLogMotionSeeds(int[] startMotion) {
    for (int motion : startMotion) {
      if (motion == Roadmap.DIR_NONE) {
        motionFellBackToStopped++;
      } else {
        motionSeededFromLastPlan++;
      }
    }
    if (++solvesSinceMotionSeedLog < MOTION_SEED_LOG_EVERY_SOLVES) {
      return;
    }
    solvesSinceMotionSeedLog = 0;
    long total = motionSeededFromLastPlan + motionFellBackToStopped;
    logger.info(
        "[MAPF motion seed] over {} solves the root state knew a vehicle was still rolling"
            + " {} of {} times ({}%); the rest started as stopped, which lets the plan turn at"
            + " the window boundary sooner than the vehicle physically can.",
        MOTION_SEED_LOG_EVERY_SOLVES, motionSeededFromLastPlan, total,
        total == 0 ? "n/a" : String.format("%.1f", 100.0 * motionSeededFromLastPlan / total));
  }

  static int[] planFromWhereTheVehicleActuallyIs(int[] plan, int start) {
    if (plan == null) {
      return null;
    }
    for (int t = 2; t < plan.length; t++) {
      if (plan[t] == start) {
        return Arrays.copyOfRange(plan, t - 1, plan.length);
      }
    }
    return plan;
  }

  private void rememberPlanPaths(
      List<String> agents, int[][] planPaths, int[] goals, boolean executable) {
    for (Map.Entry<String, Integer> e : lastPlanGoals.entrySet()) {
      if (!agents.contains(e.getKey())) {
        goalsOfPlansDroppedWhenAVehicleLeftTheSolve.put(e.getKey(), e.getValue());
      }
    }
    lastPlanPaths.keySet().retainAll(agents);
    lastPlanGoals.keySet().retainAll(agents);
    if (!executable
        || planPaths == null
        || planPaths.length != agents.size()
        || goals.length != agents.size()) {
      return;
    }
    for (int a = 0; a < agents.size(); a++) {
      lastPlanPaths.put(agents.get(a), planPaths[a].clone());
      lastPlanGoals.put(agents.get(a), goals[a]);
    }
  }

  static StaticBodies staticBodiesOf(
      Roadmap roadmap, Map<String, String> heldPoints, BodyState bodyState) {
    int held = heldPoints.size();
    if (held == 0) {
      return StaticBodies.NONE;
    }
    int[] cells = new int[held];
    int[] noses = new int[held];
    boolean[] loaded = new boolean[held];
    int body = 0;
    for (Map.Entry<String, String> vehicle : heldPoints.entrySet()) {
      cells[body] = roadmap.idOf(vehicle.getValue());
      noses[body] = bodyState.axisOf(vehicle.getKey());
      loaded[body] = bodyState.loaded(vehicle.getKey());
      body++;
    }
    return new StaticBodies(cells, noses, loaded);
  }

  private void reportGoalsThatCannotCoexist(
      List<String> agents, int[] goalVertices, boolean[] loaded) {
    if (!Footprint.enabled()) {
      return;
    }
    if (goalGeometry == null) {
      goalGeometry = new Footprint(roadmap);
    }
    for (int a = 0; a < agents.size(); a++) {
      for (int b = a + 1; b < agents.size(); b++) {
        if (goalVertices[a] == goalVertices[b]) {
          logger.error(
              "[MAPF goal-clash] {} and {} were both aimed at {}. One cell cannot hold two"
                  + " vehicles, so this fleet has no solution and the plan below is whatever the"
                  + " window happened to reach, not a route to those goals.",
              agents.get(a), agents.get(b), roadmap.nameOf(goalVertices[a]));
          continue;
        }
        if (!goalGeometry.mayInteract(goalVertices[a], goalVertices[b])) {
          continue;
        }
        if (goalGeometry.noHeadingFits(
            goalVertices[a], loaded[a], goalVertices[b], loaded[b])) {
          logger.error(
              "[MAPF goal-clash] {}@{} and {}@{} are {}mm apart and their bodies overlap at every"
                  + " heading either could arrive on, so both can never stand there at once."
                  + " The solver will spend its whole budget and still not reach these goals;"
                  + " the fix belongs where the goals are chosen, not here.",
              agents.get(a), roadmap.nameOf(goalVertices[a]),
              agents.get(b), roadmap.nameOf(goalVertices[b]),
              Math.round(Math.hypot(
                  roadmap.xOf(goalVertices[a]) - roadmap.xOf(goalVertices[b]),
                  roadmap.yOf(goalVertices[a]) - roadmap.yOf(goalVertices[b]))));
        }
      }
    }
  }

  private FleetPlan solveFleet(
      Roadmap map, MapfInstance instance, List<String> agents, Set<String> blockedVertices,
      List<String> staticVehicleNames, StaticBodies statics, SolverParams params,
      AgentPriority[] rootPriorities, int[] startAxis, boolean[] loaded, int[][] seedGuidePaths,
      int[][] committedPrefixes, int[] startMotion, LegalMoves moves) {
    int n = instance.starts().length;

    Lacam2Solver.Result result = new Lacam2Solver(map).solve(
        instance, params.deadlineMs(), params.seed(), Lacam2Solver.Objective.NONE,
        Lacam2Solver.DEFAULT_RESTART_RATE, LACAM2_LOOP_LIMIT, params.window(),
        rootPriorities, startAxis, loaded, seedGuidePaths, committedPrefixes, statics,
        startMotion, moves);
    MapfSolveCapture capture = MapfSolveCapture.capture(
        modelName, roadmap, blockedVertices, agents, instance, params,
        Lacam2Solver.Objective.NONE, Lacam2Solver.DEFAULT_RESTART_RATE,
        LACAM2_LOOP_LIMIT, rootPriorities, startAxis, loaded, seedGuidePaths,
        committedPrefixes, staticVehicleNames, statics, startMotion, moves, result);
    MapfCaptureStore.record(capture);
    dumpTriggeredCapture(capture);

    if (!result.solved()) {
      logger.error(
          "[MAPF solve] lacam2 returned NO plan (agents={} window={} loops={}/{} constrained={}"
              + " nodes={} in {}ms); reporting NOT executable so the caller keeps the previous"
              + " plan.",
          n, params.window(), result.loopCount(), LACAM2_LOOP_LIMIT_LABEL,
          result.constrainedCalls(),
          result.nodesGenerated(), result.runtimeMs());
      return new FleetPlan(
          holdInPlace(instance.starts()), false, result.executedPriorities(),
          result.guidePaths(), 1, result);
    }

    List<int[]> configs = result.solution();
    int[][] rawPaths = toAgentPaths(configs, n);
    if (searchCollapsedToSingleRollout(result, n, params.window())) {
      logger.warn(
          "[MAPF search] windowed search generated only {} node(s) for {} agents and stopped at"
              + " loop {}/{} without touching its budget: the incumbent set by the window cut-off"
              + " f-prunes the whole frontier, so this plan is ONE PIBT rollout and NO alternative"
              + " branch was ever tried. A bad rollout is shipped as-is.",
          result.nodesGenerated(), n, result.loopCount(), LACAM2_LOOP_LIMIT_LABEL);
    }
    return new FleetPlan(
        rawPaths, true, result.executedPriorities(), result.guidePaths(), configs.size(), result);
  }

  private static void dumpTriggeredCapture(MapfSolveCapture capture) {
    List<String> triggers = new ArrayList<>();
    if (!capture.result().solved()) {
      triggers.add("no-plan");
    }
    if (!capture.result().reachedGoals() && hasCellRevisit(capture.result().rawPaths())) {
      triggers.add("oscillation");
    }
    if (capture.result().pibtStepFailures() > 0) {
      triggers.add("pibt-failure");
    }
    if (capture.result().shuttleDetections() > 0) {
      triggers.add("shuttle");
    }
    for (String trigger : triggers) {
      MapfCaptureStore.dumpIfAllowed(capture, trigger);
    }
  }

  private static boolean hasCellRevisit(List<List<String>> paths) {
    for (List<String> path : paths) {
      Set<String> seen = new HashSet<>();
      String previous = null;
      for (String point : path) {
        if (!point.equals(previous) && !seen.add(point)) {
          return true;
        }
        seen.add(point);
        previous = point;
      }
    }
    return false;
  }

  private FleetPlan finalizeFleetPlan(
      Roadmap map, FleetPlan solvedFleet, List<String> agents, int[] goals, int[][] dist,
      ClaimPolicy claimPolicy, int immobileCount, int window) {
    if (!solvedFleet.executable()) {
      return solvedFleet;
    }

    List<AgentWindowPlan> plans = new ArrayList<>(agents.size());
    for (int agent = 0; agent < agents.size(); agent++) {
      plans.add(new AgentWindowPlan(
          solvedFleet.paths()[agent], goals[agent],
          claimPolicy.isRetreating(agents.get(agent))));
    }

    int[][] adjustedPaths = RetreatGoal.adjust(plans);
    GoalPathCompletion.Result completion = GoalPathCompletion.complete(
        map, adjustedPaths, goals, dist);
    logStrandedGoals(map, goals, completion.strandedAgents());
    int[][] completedPaths = completion.paths();
    Lacam2Solver.Result result = solvedFleet.solverResult();
    logger.info(
        "[MAPF solve] method=lacam2-windowed window={} agents={} immobile={} reachedGoals={}"
            + " optimal={} makespan={} loops={}/{} constrained={} nodes={} in {}ms",
        window, agents.size(), immobileCount, result.reachedGoals(), result.optimal(),
        completedPaths[0].length - 1, result.loopCount(), LACAM2_LOOP_LIMIT_LABEL,
        result.constrainedCalls(), result.nodesGenerated(), result.runtimeMs());
    return solvedFleet.withPaths(completedPaths);
  }

  private static boolean searchCollapsedToSingleRollout(
      Lacam2Solver.Result result, int agentCount, int window) {
    return agentCount > 1
        && window != SolverParams.WINDOW_FULL
        && searchStoppedWithLoopBudgetLeft(result)
        && result.nodesGenerated() <= window + 1;
  }

  private int[][] committedPrefixVertices(
      List<String> agents, Map<String, List<String>> committedPrefixes) {
    int[][] result = new int[agents.size()][];
    for (int agent = 0; agent < agents.size(); agent++) {
      List<String> prefix = committedPrefixes.getOrDefault(agents.get(agent), List.of());
      int[] vertices = new int[prefix.size()];
      for (int step = 0; step < prefix.size(); step++) {
        vertices[step] = roadmap.idOf(prefix.get(step));
      }
      result[agent] = vertices;
    }
    return result;
  }

  private Map<String, List<String>> namesOfCommittedPrefixes(
      List<String> agents, int[][] committedPrefixes) {
    Map<String, List<String>> result = new LinkedHashMap<>();
    for (int agent = 0; agent < agents.size(); agent++) {
      List<String> prefix = new ArrayList<>(committedPrefixes[agent].length);
      for (int vertex : committedPrefixes[agent]) {
        prefix.add(roadmap.nameOf(vertex));
      }
      result.put(agents.get(agent), List.copyOf(prefix));
    }
    return Map.copyOf(result);
  }

  private static boolean searchStoppedWithLoopBudgetLeft(Lacam2Solver.Result result) {
    return LACAM2_LOOP_LIMIT != Lacam2Solver.LOOP_LIMIT_UNBOUNDED
        && result.loopCount() < LACAM2_LOOP_LIMIT;
  }

  private void logStrandedGoals(Roadmap map, int[] goals, List<Integer> strandedAgents) {
    if (strandedAgents.isEmpty()) {
      return;
    }
    List<String> strandedGoals = new ArrayList<>(strandedAgents.size());
    for (int agent : strandedAgents) {
      strandedGoals.add(map.nameOf(goals[agent]));
    }
    logger.warn(
        "[MAPF tail] {} route(s) could not be completed to their goal (blocked gradient), so"
            + " they do NOT end at the goal and FMSRouter will plan those vehicles alone: {}",
        strandedGoals.size(), strandedGoals);
  }

  static int[][] toAgentPaths(List<int[]> configs, int agentCount) {
    int[][] paths = new int[agentCount][configs.size()];
    for (int t = 0; t < configs.size(); t++) {
      int[] config = configs.get(t);
      for (int a = 0; a < agentCount; a++) {
        paths[a][t] = config[a];
      }
    }
    return paths;
  }

  private static int[][] holdInPlace(int[] starts) {
    int[][] paths = new int[starts.length][1];
    for (int a = 0; a < starts.length; a++) {
      paths[a][0] = starts[a];
    }
    return paths;
  }
}
