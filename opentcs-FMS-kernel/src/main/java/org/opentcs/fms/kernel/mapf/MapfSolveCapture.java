package org.opentcs.fms.kernel.mapf;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;
import org.opentcs.fms.routing.AgentPriority;
import org.opentcs.fms.routing.Lacam2Solver;
import org.opentcs.fms.routing.LegalMoves;
import org.opentcs.fms.routing.MapfInstance;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.SolverParams;
import org.opentcs.fms.routing.StaticBodies;
import org.opentcs.fms.routing.pibt.Footprint;
import org.opentcs.fms.routing.pibt.PibtEngine;

public record MapfSolveCapture(
    int schemaVersion,
    String capturedAt,
    String modelName,
    String roadmapSha256,
    List<String> blockedVertices,
    List<AgentInput> agents,
    List<StaticBodyInput> staticBodies,
    List<List<String>> seedGuidePaths,
    List<List<String>> committedPrefixes,
    List<String> retreatGoals,
    SolverInput solverInput,
    Map<String, String> effectiveSettings,
    ResultSummary result) {

  private static final int SCHEMA_VERSION = 1;
  private static final ObjectMapper JSON = new ObjectMapper();

  public record AgentInput(
      String vehicle, String start, String goal, int startAxis, int startMotion,
      boolean loaded, AgentPriority rootPriority) {
  }

  public record StaticBodyInput(String vehicle, String cell, int nose, boolean loaded) {
  }

  public record SolverInput(
      long deadlineMs, int seed, String objective, double restartRate, int loopLimit, int window) {
  }

  public record ResultSummary(
      boolean solved, boolean reachedGoals, int loopCount, int nodesGenerated,
      int constrainedCalls, int pibtStepFailures, int shuttleDetections, long runtimeMs,
      List<List<String>> rawPaths) {
  }

  public MapfSolveCapture {
    blockedVertices = List.copyOf(blockedVertices);
    agents = List.copyOf(agents);
    staticBodies = List.copyOf(staticBodies);
    seedGuidePaths = copyNullablePaths(seedGuidePaths);
    committedPrefixes = copyNullablePaths(committedPrefixes);
    retreatGoals = copyNullableStrings(retreatGoals);
    effectiveSettings = Map.copyOf(effectiveSettings);
  }

  public static MapfSolveCapture capture(
      String modelName, Roadmap roadmap, Set<String> blockedVertices,
      List<String> agentNames, MapfInstance instance, SolverParams params,
      Lacam2Solver.Objective objective, double restartRate, int loopLimit,
      AgentPriority[] priorities, int[] startAxis, boolean[] loaded,
      int[][] seedGuidePaths, int[][] committedPrefixes,
      List<String> staticVehicleNames, StaticBodies statics, int[] startMotion,
      LegalMoves moves, Lacam2Solver.Result result) {
    List<AgentInput> agents = new ArrayList<>(agentNames.size());
    for (int agent = 0; agent < agentNames.size(); agent++) {
      agents.add(new AgentInput(
          agentNames.get(agent), roadmap.nameOf(instance.starts()[agent]),
          roadmap.nameOf(instance.goals()[agent]), startAxis[agent],
          startMotion[agent], loaded[agent], priorities[agent]));
    }
    List<StaticBodyInput> staticBodies = new ArrayList<>(statics.size());
    for (int body = 0; body < statics.size(); body++) {
      staticBodies.add(new StaticBodyInput(
          staticVehicleNames.get(body), roadmap.nameOf(statics.cells()[body]),
          statics.noses()[body], statics.loaded()[body]));
    }
    List<String> retreatGoals = new ArrayList<>();
    for (int goal : moves.retreatGoals()) {
      retreatGoals.add(goal < 0 ? null : roadmap.nameOf(goal));
    }
    return new MapfSolveCapture(
        SCHEMA_VERSION, Instant.now().toString(), modelName, roadmap.contentHash(),
        new ArrayList<>(new TreeSet<>(blockedVertices)), agents, staticBodies,
        namesOfPaths(roadmap, seedGuidePaths), namesOfPaths(roadmap, committedPrefixes),
        retreatGoals,
        new SolverInput(params.deadlineMs(), params.seed(), objective.name(), restartRate,
            loopLimit, params.window()),
        captureEffectiveSettings(), summarize(roadmap, agentNames.size(), result));
  }

  public String toJson() {
    try {
      return JSON.writerWithDefaultPrettyPrinter().writeValueAsString(this);
    } catch (JsonProcessingException e) {
      throw new IllegalStateException(e);
    }
  }

  public static MapfSolveCapture fromJson(String json) {
    try {
      return JSON.readValue(json, MapfSolveCapture.class);
    } catch (JsonProcessingException e) {
      throw new IllegalArgumentException(e);
    }
  }

  public List<List<String>> replayRawPaths(Roadmap roadmap) {
    if (!roadmapSha256.equals(roadmap.contentHash())) {
      throw new IllegalArgumentException("The supplied roadmap does not match this capture");
    }
    Set<Integer> blocked = new TreeSet<>();
    blockedVertices.forEach(name -> blocked.add(roadmap.idOf(name)));
    Roadmap planningRoadmap = blocked.isEmpty() ? roadmap : roadmap.blocking(blocked);
    int agentCount = agents.size();
    int[] starts = new int[agentCount];
    int[] goals = new int[agentCount];
    int[] axes = new int[agentCount];
    int[] motions = new int[agentCount];
    boolean[] loaded = new boolean[agentCount];
    AgentPriority[] priorities = new AgentPriority[agentCount];
    for (int agent = 0; agent < agentCount; agent++) {
      AgentInput input = agents.get(agent);
      starts[agent] = planningRoadmap.idOf(input.start());
      goals[agent] = planningRoadmap.idOf(input.goal());
      axes[agent] = input.startAxis();
      motions[agent] = input.startMotion();
      loaded[agent] = input.loaded();
      priorities[agent] = input.rootPriority();
    }
    StaticBodies bodies = staticBodiesFor(planningRoadmap);
    int[][] guide = pathsOf(planningRoadmap, seedGuidePaths);
    int[][] prefixes = pathsOf(planningRoadmap, committedPrefixes);
    int[] retreats = retreatGoals.isEmpty() ? null : idsOf(planningRoadmap, retreatGoals);
    Lacam2Solver.Result replayed = new Lacam2Solver(planningRoadmap).solve(
        new MapfInstance(starts, goals), 0, solverInput.seed(),
        Lacam2Solver.Objective.valueOf(solverInput.objective()), solverInput.restartRate(),
        result.loopCount(), solverInput.window(), priorities, axes, loaded, guide, prefixes,
        bodies, motions, new LegalMoves(planningRoadmap, retreats));
    return namesOfSolution(planningRoadmap, agentCount, replayed);
  }

  private StaticBodies staticBodiesFor(Roadmap roadmap) {
    int[] cells = new int[staticBodies.size()];
    int[] noses = new int[staticBodies.size()];
    boolean[] loaded = new boolean[staticBodies.size()];
    for (int body = 0; body < staticBodies.size(); body++) {
      StaticBodyInput input = staticBodies.get(body);
      cells[body] = roadmap.idOf(input.cell());
      noses[body] = input.nose();
      loaded[body] = input.loaded();
    }
    return new StaticBodies(cells, noses, loaded);
  }

  private static ResultSummary summarize(
      Roadmap roadmap, int agentCount, Lacam2Solver.Result result) {
    return new ResultSummary(
        result.solved(), result.reachedGoals(), result.loopCount(), result.nodesGenerated(),
        result.constrainedCalls(), result.pibtStepFailures(), result.shuttleDetections(),
        result.runtimeMs(), namesOfSolution(roadmap, agentCount, result));
  }

  private static List<List<String>> namesOfSolution(
      Roadmap roadmap, int agentCount, Lacam2Solver.Result result) {
    if (!result.solved()) {
      return emptyPaths(agentCount);
    }
    List<List<String>> paths = new ArrayList<>(agentCount);
    for (int agent = 0; agent < agentCount; agent++) {
      List<String> path = new ArrayList<>(result.solution().size());
      for (int[] config : result.solution()) {
        path.add(roadmap.nameOf(config[agent]));
      }
      paths.add(List.copyOf(path));
    }
    return List.copyOf(paths);
  }

  private static List<List<String>> namesOfPaths(Roadmap roadmap, int[][] paths) {
    if (paths == null) {
      return List.of();
    }
    List<List<String>> names = new ArrayList<>(paths.length);
    for (int[] path : paths) {
      if (path == null) {
        names.add(null);
        continue;
      }
      List<String> points = new ArrayList<>(path.length);
      for (int vertex : path) {
        points.add(roadmap.nameOf(vertex));
      }
      names.add(List.copyOf(points));
    }
    return copyNullablePaths(names);
  }

  private static int[][] pathsOf(Roadmap roadmap, List<List<String>> paths) {
    if (paths.isEmpty()) {
      return null;
    }
    int[][] vertices = new int[paths.size()][];
    for (int path = 0; path < paths.size(); path++) {
      List<String> points = paths.get(path);
      if (points == null) {
        continue;
      }
      vertices[path] = idsOf(roadmap, points);
    }
    return vertices;
  }

  private static int[] idsOf(Roadmap roadmap, List<String> names) {
    int[] ids = new int[names.size()];
    for (int index = 0; index < names.size(); index++) {
      ids[index] = roadmap.idOf(names.get(index));
    }
    return ids;
  }

  private static List<List<String>> emptyPaths(int count) {
    List<List<String>> paths = new ArrayList<>(count);
    for (int agent = 0; agent < count; agent++) {
      paths.add(List.of());
    }
    return List.copyOf(paths);
  }

  private static List<List<String>> copyNullablePaths(List<List<String>> paths) {
    List<List<String>> copy = new ArrayList<>(paths.size());
    for (List<String> path : paths) {
      copy.add(path == null ? null : List.copyOf(path));
    }
    return Collections.unmodifiableList(copy);
  }

  private static List<String> copyNullableStrings(List<String> values) {
    return Collections.unmodifiableList(new ArrayList<>(values));
  }

  private static Map<String, String> captureEffectiveSettings() {
    TreeMap<String, String> settings = new TreeMap<>();
    System.getProperties().stringPropertyNames().stream()
        .filter(MapfSolveCapture::isSolveProperty)
        .forEach(name -> settings.put(name, System.getProperty(name)));
    settings.putAll(Footprint.effectiveSettings());
    settings.putAll(PibtEngine.effectiveSettings());
    settings.putAll(Roadmap.effectiveSettings());
    settings.putAll(Lacam2Solver.effectiveSettings());
    putDefault(settings, "fms.guide.on", "true");
    putDefault(settings, "fms.guide.profile", "fms");
    putDefault(settings, "fms.guide.window", "20");
    putDefault(settings, "fms.guide.numRefine", "2");
    putDefault(settings, "fms.guide.collisionCost", "3.0");
    putDefault(settings, "fms.guide.collisionCostOrder", "1e-7");
    putDefault(settings, "fms.guide.footprintCost", "2.0");
    putDefault(settings, "fms.guide.restingBodyTicks", "2");
    putDefault(settings, "fms.guide.deterministic", "false");
    putDefault(settings, "fms.guide.clearGoalFirst", "true");
    putDefault(settings, "fms.guide.reserveFootprint", "true");
    putDefault(settings, "fms.guide.traceRolloutCarry", "true");
    putDefault(settings, "fms.lllg.window", "10");
    putDefault(settings, "fms.lllg.numRefine", "1");
    putDefault(settings, "fms.lllg.collisionCost", "1.0");
    putDefault(settings, "fms.lllg.collisionCostOrder", "1e-7");
    putDefault(settings, "fms.lllg.deterministic", "false");
    putDefault(settings, "fms.lllg.clearGoalFirst", "false");
    settings.put("effective.fms.lllg.window", Integer.toString(Math.max(
        2, Integer.getInteger("fms.lllg.window", 10))));
    putDefault(settings, "fms.mapf.lacam2LoopLimit", "unbounded");
    putDefault(settings, "fms.mapf.window", "10");
    putDefault(settings, "fms.mapf.minSolveGapMs", "1000");
    putDefault(settings, "fms.mapf.commitMs", "4000");
    putDefault(settings, "fms.mapf.stallDumpSeconds", "25");
    putDefault(settings, "fms.mapf.quarterTurnCost", "1");
    putDefault(settings, "fms.mapf.dropOffLaneWeight", "4");
    putDefault(settings, "fms.mapf.cargoSlotWeight", "0");
    putDefault(settings, "fms.pibt.clearanceMm", "0");
    putDefault(settings, "fms.pibt.hindrance", "true");
    putDefault(settings, "fms.pibt.traceRefusals", "true");
    putDefault(settings, "fms.pibt.logGuidedRefusals", "false");
    putDefault(settings, "fms.pibt.guidedRefusalLines", "20");
    putDefault(settings, "fms.pibt.vehicleLengthMm", "840");
    putDefault(settings, "fms.pibt.vehicleWidthMm", "425");
    putDefault(settings, "fms.pibt.cargoLengthMm", "800");
    putDefault(settings, "fms.pibt.cargoWidthMm", "800");
    return Map.copyOf(settings);
  }

  private static void putDefault(Map<String, String> settings, String property, String defaultValue) {
    settings.putIfAbsent(property, System.getProperty(property, defaultValue));
  }

  private static boolean isSolveProperty(String name) {
    return name.startsWith("fms.pibt.")
        || name.startsWith("fms.mapf.")
        || name.startsWith("fms.guide.")
        || name.startsWith("fms.lllg.");
  }
}
