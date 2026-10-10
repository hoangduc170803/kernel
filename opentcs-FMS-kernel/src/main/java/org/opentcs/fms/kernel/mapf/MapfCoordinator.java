package org.opentcs.fms.kernel.mapf;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.LongSupplier;

import org.opentcs.components.kernel.services.PlantModelService;
import org.opentcs.data.model.PlantModel;
import org.opentcs.fms.kernel.mapf.adg.Adg;
import org.opentcs.fms.kernel.mapf.adg.AdgDiagnostics;
import org.opentcs.fms.kernel.mapf.adg.AdgService;
import org.opentcs.fms.kernel.mapf.adg.Step;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.pibt.Footprint;

import jakarta.inject.Inject;
import jakarta.inject.Singleton;

@Singleton
public class MapfCoordinator {
  private static final org.slf4j.Logger logger = org.slf4j.LoggerFactory.getLogger(MapfCoordinator.class);

  private final Map<String, String> lastHeldBack = new HashMap<>();
  private final PlantModelService plantModelService;
  private final AdgService adgService;

  public record PlannedRoute(
      List<String> points, String requestedGoal, String plannedGoal,
      List<String> solverCommittedPrefix) {
    public PlannedRoute(List<String> points, String requestedGoal, String plannedGoal) {
      this(points, requestedGoal, plannedGoal, List.of());
    }

    public Optional<List<String>> tailFrom(String from) {
      int committedEnd = endOfCommittedPrefix();
      int idx = indexAtOrAfter(from, committedEnd);
      if (idx < 0) {
        idx = points.indexOf(from);
      }
      if (idx < 0 || points.size() - idx < 2) {
        return Optional.empty();
      }
      return Optional.of(points.subList(idx, points.size()));
    }

    private int endOfCommittedPrefix() {
      if (solverCommittedPrefix == null || solverCommittedPrefix.isEmpty()
          || solverCommittedPrefix.size() > points.size()
          || !points.subList(0, solverCommittedPrefix.size()).equals(solverCommittedPrefix)) {
        return 0;
      }
      return solverCommittedPrefix.size() - 1;
    }

    private int indexAtOrAfter(String point, int from) {
      for (int index = from; index < points.size(); index++) {
        if (points.get(index).equals(point)) {
          return index;
        }
      }
      return -1;
    }
  }

  public record DriveRoute(List<String> points, List<String> solverCommittedPrefix) {
  }

  public record DriveOrderLeg(int number, int total) {
    public boolean isFirst() {
      return number == 1;
    }
  }

  public record SolveRequest(
      RoutingContext ctx,
      Map<String, String> positions,
      Map<String, String> goals,
      Set<String> immobile,
      ClaimPolicy claimPolicy,
      BodyState bodyState,
      Map<String, List<String>> committedPrefixes,
      List<Footprint.Obstacle> obstacles,
      long startedAtMs) {
  }

  public record StalePlanState(boolean solveOwed, long changedAtMs, List<String> changedVehicles) {
  }

  private volatile RoutingContext context;
  private volatile Map<String, PlannedRoute> routeCache = Map.of();
  private volatile Map<String, String> currentGoals = Map.of();
  private volatile Set<String> currentImmobile = Set.of();
  private volatile Map<String, String> installedGoals = Map.of();
  private volatile Set<String> installedImmobile = Set.of();
  private volatile long currentSnapshotChangedAtMs;
  private final AtomicBoolean resolving = new AtomicBoolean(false);
  private final ReplanPolicy replanPolicy = new ReplanPolicy();
  private volatile LongSupplier clock = System::currentTimeMillis;
  private int consecutiveSolveFailures;

  @Inject
  public MapfCoordinator(
      PlantModelService plantModelService, AdgService adgService) {
    this.plantModelService = plantModelService;
    this.adgService = adgService;
  }

  public synchronized void invalidateModel() {
    context = null;
    routeCache = Map.of();
    installedGoals = Map.of();
    installedImmobile = Set.of();
    adgService.invalidate();
  }

  public RoutingContext context() {
    RoutingContext current = context;
    if (current != null) {
      return current;
    }
    synchronized (this) {
      if (context != null) {
        return context;
      }
      try {
        PlantModel pm = plantModelService.getPlantModel();
        Roadmap roadmap = PlantModelAdapter.toRoadmap(pm);
        RoutingContext built = new RoutingContext(pm.getName(), roadmap, new RouteBuilder(pm),
            new MapfPlanner(roadmap, pm.getName()));
        context = built;
        logger.info("[MAPF] roadmap ready for model '{}': {} points (local-order precedence / TPG)",
            pm.getName(), roadmap.size());
        return built;
      } catch (RuntimeException e) {
        logger.warn("[MAPF] roadmap build failed; routing falls back to default: {}", e.toString());
        return null;
      }
    }
  }

  public List<String> planSingle(
      String vehicle, String source, String goal, ClaimPolicy claimPolicy) {
    RoutingContext ctx = context();
    return ctx == null
        ? List.of()
        : ctx.planSingle(vehicle, source, goal, claimPolicy, obstacles());
  }

  public void setClock(LongSupplier clock) {
    this.clock = clock;
  }

  private long now() {
    return clock.getAsLong();
  }

  public boolean resolve(Map<String, String> positions, Map<String, String> goals) {
    return resolve(positions, goals, Set.of());
  }

  public boolean resolve(
      Map<String, String> positions, Map<String, String> goals, Set<String> immobile) {
    return resolve(positions, goals, immobile, ClaimPolicy.DEFAULT);
  }

  public boolean resolve(
      Map<String, String> positions, Map<String, String> goals, Set<String> immobile,
      ClaimPolicy claimPolicy) {
    return resolve(positions, goals, immobile, claimPolicy, BodyState.UNKNOWN);
  }

  public boolean resolve(
      Map<String, String> positions, Map<String, String> goals, Set<String> immobile,
      ClaimPolicy claimPolicy, BodyState bodyState) {
    SolveRequest request = prepare(positions, goals, immobile, claimPolicy, bodyState);
    if (request == null) {
      return false;
    }
    MapfPlanner.PlanResult result;
    try {
      result = compute(request);
    } catch (RuntimeException | Error e) {
      abortSolve("the solver threw " + e);
      throw e;
    }
    return install(request, result);
  }

  public SolveRequest prepare(
      Map<String, String> positions, Map<String, String> goals, Set<String> immobile,
      ClaimPolicy claimPolicy, BodyState bodyState) {
    return prepare(positions, goals, immobile, claimPolicy, bodyState, Map.of());
  }

  public SolveRequest prepare(
      Map<String, String> positions, Map<String, String> goals, Set<String> immobile,
      ClaimPolicy claimPolicy, BodyState bodyState,
      Map<String, List<String>> committedPrefixes) {
    if (positions.isEmpty()) {
      return null;
    }
    observeSnapshot(goals, immobile);
    if (!resolving.compareAndSet(false, true)) {
      return null;
    }
    boolean handedOff = false;
    try {
      long now = now();
      List<Footprint.Obstacle> obstacles = obstacles();
      replanPolicy.observeObstacles(obstacles);
      if (!replanPolicy.shouldSolve(positions, goals, immobile, routeCache, now)) {
        adgService.touch(now);
        return null;
      }
      RoutingContext ctx = context();
      if (ctx == null) {
        return null;
      }
      handedOff = true;
      return new SolveRequest(
          ctx, positions, goals, immobile, claimPolicy, bodyState, committedPrefixes, obstacles,
          now);
    } finally {
      if (!handedOff) {
        resolving.set(false);
      }
    }
  }

  public MapfPlanner.PlanResult compute(SolveRequest request) {
    try {
      return request.ctx().planAll(
          request.positions(), request.goals(), request.immobile(),
          request.claimPolicy(), request.bodyState(), request.committedPrefixes(),
          request.obstacles());
    } finally {
      replanPolicy.recordSolveEnded(now());
    }
  }

  public void abortSolve(String reason) {
    logger.warn("[MAPF solve] abandoned: {}", reason);
    resolving.set(false);
  }

  public boolean install(SolveRequest request, MapfPlanner.PlanResult result) {
    try {
      String outdated = outdatedBy(request);
      if (outdated != null) {
        logger.info(
            "[MAPF solve] discarding the plan solved {}ms ago: {}. The committed plan stands"
                + " until the next solve.",
            now() - request.startedAtMs(), outdated);
        return false;
      }
      if (!result.executable()) {
        consecutiveSolveFailures++;
        Adg kept = adgService.adg();
        long staleMs = kept.builtAtMs() == 0L
            ? -1L
            : now() - kept.builtAtMs();
        logger.error(
            "[MAPF solve] no usable plan ({} consecutive); KEEPING generation {}, now {}ms old."
                + " While this persists NO vehicle is rerouted and FMSRouter serves"
                + " single-vehicle paths that ignore the rest of the fleet.",
            consecutiveSolveFailures, kept.generation(), staleMs);
        return false;
      }
      consecutiveSolveFailures = 0;
      Map<String, String> goals = request.goals();
      routeCache = snapshot(
          result.routes(), goals, result.plannedGoals(), result.solverCommittedPrefixes());
      installedGoals = Map.copyOf(goals);
      installedImmobile = Set.copyOf(request.immobile());
      Adg replaced = adgService.install(
          result.spaceTimePaths(), result.coordinatedTicks(), request.ctx().roadmap(),
          request.ctx().footprint(), request.bodyState(), request.startedAtMs());
      logCoverage(adgService.adg());
      AdgDiagnostics.log(replaced, adgService.adg());
      if (logger.isDebugEnabled()) {
        long moving = routeCache.values().stream().filter(r -> r.points().size() > 1).count();
        logger.debug("[MAPF] joint solve: {} vehicles in, {} routes cached ({} MOVING >1pt)",
            request.positions().size(), routeCache.size(), moving);
      }
      replanPolicy.recordSolved(goals, request.immobile(), request.obstacles());
      return true;
    } finally {
      resolving.set(false);
    }
  }

  public boolean allocationPlanIsStale() {
    return !routeCache.isEmpty()
        && (!currentGoals.equals(installedGoals) || !currentImmobile.equals(installedImmobile));
  }

  public StalePlanState stalePlanState() {
    Set<String> changed = new TreeSet<>();
    changed.addAll(currentGoals.keySet());
    changed.addAll(installedGoals.keySet());
    changed.addAll(currentImmobile);
    changed.addAll(installedImmobile);
    changed.removeIf(vehicle -> java.util.Objects.equals(
        currentGoals.get(vehicle), installedGoals.get(vehicle))
        && currentImmobile.contains(vehicle) == installedImmobile.contains(vehicle));
    return new StalePlanState(allocationPlanIsStale(), currentSnapshotChangedAtMs,
        List.copyOf(changed));
  }

  private void observeSnapshot(Map<String, String> goals, Set<String> immobile) {
    if (!goals.equals(currentGoals) || !immobile.equals(currentImmobile)) {
      currentGoals = Map.copyOf(goals);
      currentImmobile = Set.copyOf(immobile);
      currentSnapshotChangedAtMs = now();
    }
  }

  private List<Footprint.Obstacle> obstacles() {
    return EnvironmentalObstacles.respectedIn(plantModelService);
  }

  private String outdatedBy(SolveRequest request) {
    return context == request.ctx() ? null : "the plant model was reloaded";
  }

  private void logCoverage(Adg adg) {
    StringBuilder cov = new StringBuilder();
    int windowed = 0;
    for (Map.Entry<String, List<String>> e : adg.routes().entrySet()) {
      int adgLen = e.getValue().size();
      PlannedRoute full = routeCache.get(e.getKey());
      int fullLen = full == null ? 0 : full.points().size();
      if (adgLen < fullLen) {
        windowed++;
      }
      cov.append(e.getKey()).append('=').append(adgLen).append('/').append(fullLen).append(' ');
    }
    List<String> edges = new ArrayList<>();
    adg.dependencies().forEach((waiter, blockers) -> {
      for (Step blocker : blockers) {
        edges.add(waiter + "@" + adg.pointOf(waiter)
            + "<-" + blocker + "@" + adg.pointOf(blocker));
      }
    });
    edges.sort(Comparator.naturalOrder());
    logger.info("[MAPF cov] adg/full: {}| {} windowed | {} edges: {}",
        cov, windowed, edges.size(), edges);
  }

  public Adg adg() {
    return adgService.adg();
  }

  public Optional<String> plannedGoalOf(String vehicle) {
    PlannedRoute planned = routeCache.get(vehicle);
    return planned == null ? Optional.empty() : Optional.ofNullable(planned.plannedGoal());
  }

  public Optional<List<String>> pointsFor(
      String vehicle, String from, String goal, ClaimPolicy claimPolicy) {
    return points(vehicle, from, goal, null, claimPolicy).map(DriveRoute::points);
  }

  public Optional<List<String>> pointsToDrive(
      String vehicle, String from, String goal, DriveOrderLeg leg, ClaimPolicy claimPolicy) {
    return points(vehicle, from, goal, leg, claimPolicy).map(DriveRoute::points);
  }

  public Optional<DriveRoute> pointsWithCommitmentToDrive(
      String vehicle, String from, String goal, DriveOrderLeg leg, ClaimPolicy claimPolicy) {
    return points(vehicle, from, goal, leg, claimPolicy);
  }

  private Optional<DriveRoute> points(
      String vehicle, String from, String goal, DriveOrderLeg legToAnnounce,
      ClaimPolicy claimPolicy) {
    boolean announce = legToAnnounce != null;
    RoutingContext ctx = context();
    if (ctx == null) {
      return Optional.empty();
    }
    PlannedRoute planned = routeCache.get(vehicle);
    Optional<List<String>> tail = planned == null ? Optional.empty() : planned.tailFrom(from);
    if (tail.isEmpty()) {
      return alone(
          ctx, vehicle, from, goal, unusableReason(planned, from), legToAnnounce, claimPolicy);
    }
    if (!goal.equals(planned.requestedGoal())) {
      return alone(ctx, vehicle, from, goal,
          "fleet plan was solved for goal " + planned.requestedGoal() + ", not " + goal
              + " (the goal changed since that solve)", legToAnnounce, claimPolicy);
    }
    List<String> remainder = tail.get();
    String end = remainder.get(remainder.size() - 1);
    String plannedGoal = planned.plannedGoal();
    if (goal.equals(plannedGoal)) {
      if (goal.equals(end)) {
        return Optional.of(new DriveRoute(remainder, planned.solverCommittedPrefix()));
      }
      if (announce) {
        logger.info(
            "[MAPF router] {} {}->{}: fleet plan aimed at the goal but its route stops at {};"
                + " driving the COORDINATED prefix ({} pts) and completing {}->{} alone. The"
                + " uncoordinated part sits past the planning window and is replaced before the"
                + " vehicle gets there.",
            vehicle, from, goal, end, remainder.size(), end, goal);
      }
      return plannedDriveRoute(
          completed(ctx, remainder, vehicle, goal, claimPolicy), planned.solverCommittedPrefix());
    }
    int goalOnRoute = remainder.indexOf(goal);
    if (goalOnRoute >= 0) {
      List<String> upToGoal = remainder.subList(0, goalOnRoute + 1);
      if (announce) {
        logger.info(
            "[MAPF router] {} {}->{}: fleet plan aimed at the surrogate {} beyond the goal;"
                + " driving the COORDINATED route CUT at {} ({} of {} pts) so the vehicle stops"
                + " on its destination instead of overshooting and reversing back into it.",
            vehicle, from, goal, plannedGoal, goal, upToGoal.size(), remainder.size());
      }
      return plannedDriveRoute(upToGoal, planned.solverCommittedPrefix());
    }
    if (announce) {
      logger.info(
          "[MAPF router] {} {}->{}: fleet plan aimed at the surrogate {}; driving the COORDINATED"
              + " prefix ({} pts) and completing {}->{} alone. The completion is uncoordinated but"
              + " the prefix keeps the vehicle out of the oncoming traffic the fleet plan saw.",
          vehicle, from, goal, plannedGoal, remainder.size(), end, goal);
    }
    return plannedDriveRoute(
        completed(ctx, remainder, vehicle, goal, claimPolicy), planned.solverCommittedPrefix());
  }

  private Optional<DriveRoute> alone(
      RoutingContext ctx, String vehicle, String from, String goal, String reason,
      DriveOrderLeg legToAnnounce, ClaimPolicy claimPolicy) {
    if (legToAnnounce == null || legToAnnounce.isFirst()) {
      String episode = from + "->" + goal + "|" + reason;
      if (!episode.equals(lastHeldBack.get(vehicle))) {
        lastHeldBack.put(vehicle, episode);
        logger.warn(
            "[MAPF router] {} {}->{}: NO coordinated route in a {}-vehicle fleet plan ({});"
                + " holding the order back until a solve covers this vehicle.",
            vehicle, from, goal, routeCache.size(), reason);
      }
      return Optional.empty();
    }

    logger.debug(
        "[MAPF router] {} {}->{}: leg {} of {} planned ALONE, as expected: the fleet plan only"
            + " covers the current drive order, so no later leg can have a coordinated route"
            + " ({}).",
        vehicle, from, goal, legToAnnounce.number(), legToAnnounce.total(), reason);
    return plannedDriveRoute(
        ctx.planSingle(vehicle, from, goal, claimPolicy, obstacles()), List.of());
  }

  private static Optional<DriveRoute> plannedDriveRoute(
      List<String> points, List<String> solverCommittedPrefix) {
    return points.size() < 2
        ? Optional.empty()
        : Optional.of(new DriveRoute(points, solverCommittedPrefix));
  }

  private static String unusableReason(PlannedRoute planned, String from) {
    if (planned == null) {
      return "vehicle absent from the fleet plan";
    }
    List<String> points = planned.points();
    if (points.size() < 2) {
      return "fleet plan pinned it at " + points.get(0)
          + " (held/at-goal when the fleet was solved)";
    }
    String span = "fleet plan routed it " + points.get(0) + ".." + points.get(points.size() - 1);
    if (points.indexOf(from) >= 0) {
      return span + ", which ends at " + from + " with nothing left to drive";
    }
    return span + ", which does not pass through " + from;
  }

  private List<String> completed(
      RoutingContext ctx, List<String> head, String vehicle, String goal, ClaimPolicy claimPolicy) {
    List<String> completion = ctx.planSingle(
        vehicle, head.get(head.size() - 1), goal, claimPolicy, obstacles());
    if (completion.size() < 2) {
      return List.of();
    }
    List<String> points = new ArrayList<>(head);
    points.addAll(completion.subList(1, completion.size()));
    return points;
  }

  public Optional<List<String>> cachedRoute(String vehicle, String goal) {
    PlannedRoute planned = routeCache.get(vehicle);
    List<String> route = planned == null ? null : planned.points();
    boolean hit = route != null && !route.isEmpty() && route.get(route.size() - 1).equals(goal);
    if (logger.isDebugEnabled()) {
      logger.debug("[MAPF cache] {} goal={}: {} (cached={}, first={}, last={})",
          vehicle, goal,
          hit ? "HIT" : (route == null ? "MISS-no-entry" : "MISS-goal"),
          route == null ? "none" : route.size() + "pts",
          (route == null || route.isEmpty()) ? "-" : route.get(0),
          (route == null || route.isEmpty()) ? "-" : route.get(route.size() - 1));
    }
    if (hit) {
      return Optional.of(route);
    }
    return Optional.empty();
  }

  private static Map<String, PlannedRoute> snapshot(
      Map<String, List<String>> routes, Map<String, String> requestedGoals,
      Map<String, String> plannedGoals, Map<String, List<String>> solverCommittedPrefixes) {
    Map<String, PlannedRoute> out = new HashMap<>();
    for (Map.Entry<String, List<String>> e : routes.entrySet()) {
      List<String> points = e.getValue();
      if (e.getKey() == null || points == null || points.isEmpty()) {
        continue;
      }
      out.put(e.getKey(), new PlannedRoute(
          List.copyOf(points),
          requestedGoals.get(e.getKey()),
          plannedGoals.getOrDefault(e.getKey(), points.get(points.size() - 1)),
          List.copyOf(solverCommittedPrefixes.getOrDefault(e.getKey(), List.of()))));
    }
    return Map.copyOf(out);
  }
}
