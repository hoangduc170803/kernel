package org.opentcs.fms.kernel.mapf;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.time.Instant;
import java.util.function.LongSupplier;

import org.opentcs.components.kernel.Scheduler;
import org.opentcs.components.kernel.services.InternalPlantModelService;
import org.opentcs.data.TCSObjectReference;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.TCSResource;
import org.opentcs.data.model.Vehicle;
import org.opentcs.fms.kernel.mapf.adg.Adg;
import org.opentcs.fms.kernel.mapf.adg.AdgService;
import org.opentcs.fms.kernel.mapf.adg.Step;
import org.opentcs.fms.kernel.mapf.MapfCoordinator.StalePlanState;

import jakarta.inject.Inject;
import jakarta.inject.Singleton;

@Singleton
public class MapfPrecedenceModule
    implements
      Scheduler.Module {

  private static final int OFF_PLAN = -1;
  private static final long MAX_PLAN_AGE_MS = 30_000L;
  private static final org.slf4j.Logger logger =
      org.slf4j.LoggerFactory.getLogger(MapfPrecedenceModule.class);

  private record StepMismatchEpisode(
      long generation, String position, String requested, int mappedStep, String mappedPoint) {
  }

  private final AdgService adgService;
  private final InternalPlantModelService plantModelService;
  private final MapfCoordinator coordinator;
  private final VetoLogger vetoLog = new VetoLogger();

  private volatile boolean initialized;
  private volatile LongSupplier millisClock = System::currentTimeMillis;

  private final Map<String, List<String>> claims = new HashMap<>();
  private final Map<String, Integer> lastKnownStep = new HashMap<>();
  private final Map<String, Long> staleAllocationGenerationByVehicle = new HashMap<>();
  private final Map<String, StepMismatchEpisode> stepMismatchByVehicle = new HashMap<>();

  @Inject
  public MapfPrecedenceModule(
      AdgService adgService, InternalPlantModelService plantModelService,
      MapfCoordinator coordinator) {
    this.adgService = adgService;
    this.plantModelService = plantModelService;
    this.coordinator = coordinator;
  }

  public MapfPrecedenceModule(
      AdgService adgService, InternalPlantModelService plantModelService) {
    this(adgService, plantModelService, null);
  }

  public void setClock(LongSupplier nanoClock) {
    vetoLog.setClock(nanoClock);
  }

  public void setMillisClock(LongSupplier millisClock) {
    this.millisClock = millisClock;
  }

  public synchronized List<String> openVetoes() {
    return vetoLog.openEpisodes();
  }

  @Override
  public void initialize() {
    initialized = true;
  }

  @Override
  public boolean isInitialized() {
    return initialized;
  }

  @Override
  public void terminate() {
    initialized = false;
  }

  @Override
  public synchronized void setAllocationState(
      Scheduler.Client client, Set<TCSResource<?>> alloc, List<Set<TCSResource<?>>> remaining) {
    String vehicle = vehicleOf(client);
    if (vehicle == null) {
      return;
    }
    List<String> route = new ArrayList<>();
    for (Set<TCSResource<?>> step : remaining) {
      for (TCSResource<?> resource : step) {
        if (resource instanceof Point) {
          String point = resource.getName();
          if (route.isEmpty() || !route.get(route.size() - 1).equals(point)) {
            route.add(point);
          }
        }
      }
    }
    claims.put(vehicle, route);
  }

  @Override
  public void prepareAllocation(Scheduler.Client client, Set<TCSResource<?>> resources) {
  }

  @Override
  public boolean hasPreparedAllocation(Scheduler.Client client, Set<TCSResource<?>> resources) {
    return true;
  }

  @Override
  public synchronized void allocationReleased(
      Scheduler.Client client, Set<TCSResource<?>> resources) {
    String vehicle = vehicleOf(client);
    if (vehicle != null) {
      vetoLog.released(vehicle);
    }
  }

  @Override
  public synchronized boolean mayAllocate(Scheduler.Client client, Set<TCSResource<?>> resources) {
    if (!initialized) {
      return true;
    }
    Adg adg = adgService.adg();
    boolean stalePlan = coordinator != null && coordinator.allocationPlanIsStale();
    String vehicle = vehicleOf(client);
    if (vehicle != null && requestsPointOutsideProvenRoute(vehicle, resources, adg)) {
      return false;
    }
    Optional<PlanProgress> progress = vehicle == null
        ? Optional.empty()
        : progressOf(vehicle, adg);
    if (vehicle != null && requestLeftThePlan(vehicle, progress, resources, adg)) {
      if (vetoLog.refusedOffPlan(vehicle)) {
        vetoLog.describeOffPlanHold(adg, vehicle, currentPositions().get(vehicle),
            pointsIn(resources), claims.get(vehicle));
      }
      return false;
    }
    Step wanted = vehicle == null
        ? null
        : new Step(vehicle, progress.map(PlanProgress::step).orElse(OFF_PLAN));
    String nextPoint = progress.map(PlanProgress::pointUnderRequest).orElse(null);
    boolean asksForItsNextStep = nextPoint != null && requests(resources, nextPoint);

    if (asksForItsNextStep && beyondTheProvenPlan(adg, wanted)) {
      if (vetoLog.refusedBeyondHorizon(vehicle)) {
        vetoLog.describeHorizonHold(adg, wanted, nextPoint);
      }
      return false;
    }
    if (adg.hasNothingToEnforce()) {
      vetoLog.noGraph();
      logStaleAllocation(adg, vehicle, resources, stalePlan);
      return true;
    }
    if (vetoLog.startedNewGeneration(adg)) {
      lastKnownStep.clear();
      stepMismatchByVehicle.clear();
    }

    if (vehicle == null) {
      return true;
    }
    Map<String, String> positions = currentPositions();
    if (!asksForItsNextStep) {
      String standingAt = positions.get(vehicle);
      String requestedPoint = firstPoint(resources);
      if (nextPoint != null && requestedPoint != null && !nextPoint.equals(requestedPoint)) {
        logStepMismatch(adg, vehicle, standingAt, requestedPoint, wanted, nextPoint);
      } else {
        stepMismatchByVehicle.remove(vehicle);
      }
      vetoLog.askedForAnotherPoint(adg, wanted, nextPoint, standingAt,
          pointsIn(resources), lastKnownStep.getOrDefault(vehicle, OFF_PLAN),
          claims.getOrDefault(vehicle, List.of()));
      logStaleAllocation(adg, vehicle, resources, stalePlan);
      return true;
    }

    Step blocker = firstBlockerStillOnItsStep(adg, wanted, positions);
    if (blocker == null) {
      vetoLog.noBlocker(vehicle);
      stepMismatchByVehicle.remove(vehicle);
      logStaleAllocation(adg, vehicle, resources, stalePlan);
      return true;
    }
    if (vetoLog.refused(wanted, blocker)) {
      vetoLog.describeHold(adg, wanted, blocker,
          positions.get(vehicle), positions.get(blocker.vehicle()),
          nextStepOf(blocker.vehicle(), adg, positions),
          lastKnownStep.getOrDefault(blocker.vehicle(), OFF_PLAN));
    }
    return false;
  }

  private void logStaleAllocation(
      Adg adg, String vehicle, Set<TCSResource<?>> resources, boolean stalePlan) {
    if (!stalePlan || vehicle == null || !logger.isInfoEnabled()
        || staleAllocationGenerationByVehicle.getOrDefault(vehicle, Long.MIN_VALUE)
            == adg.generation()) {
      return;
    }
    String point = firstPoint(resources);
    if (point == null) {
      return;
    }
    StalePlanState state = coordinator.stalePlanState();
    if (!state.solveOwed()) {
      return;
    }
    staleAllocationGenerationByVehicle.put(vehicle, adg.generation());
    logger.info(
        "[MAPF stale-alloc] {} granted {} from plan@{} goals/immobile changed at {} ({})"
            + ", solve owed",
        vehicle, point, Instant.ofEpochMilli(adg.builtAtMs()),
        Instant.ofEpochMilli(state.changedAtMs()), state.changedVehicles());
  }

  private void logStepMismatch(
      Adg adg, String vehicle, String standingAt, String requestedPoint, Step mapped,
      String mappedPoint) {
    if (!logger.isWarnEnabled()) {
      return;
    }
    int positionIndex = routeIndexOf(adg.routeOf(vehicle), standingAt,
        lastKnownStep.getOrDefault(vehicle, 0));
    StepMismatchEpisode episode = new StepMismatchEpisode(
        adg.generation(), standingAt, requestedPoint, mapped.step(), mappedPoint);
    if (episode.equals(stepMismatchByVehicle.get(vehicle))) {
      return;
    }
    stepMismatchByVehicle.put(vehicle, episode);
    logger.warn(
        "[MAPF step-mismatch] {} standing at {} requested {} but mapped to ADG step {} ({})"
            + " | anchor=claims positionIndex={}",
        vehicle, standingAt, requestedPoint, mapped.step(), mappedPoint, positionIndex);
  }

  private static String firstPoint(Set<TCSResource<?>> resources) {
    for (TCSResource<?> resource : resources) {
      if (resource instanceof Point) {
        return resource.getName();
      }
    }
    return null;
  }

  private static int routeIndexOf(List<String> route, String position, int start) {
    if (position == null) {
      return OFF_PLAN;
    }
    for (int index = Math.max(start, 0); index < route.size(); index++) {
      if (position.equals(route.get(index))) {
        return index;
      }
    }
    return OFF_PLAN;
  }

  private boolean beyondTheProvenPlan(Adg adg, Step wanted) {
    return !planIsTooOldToTrust(adg)
        && wanted.step() > adg.coordinatedStepOf(wanted.vehicle());
  }

  private boolean requestsPointOutsideProvenRoute(
      String vehicle, Set<TCSResource<?>> resources, Adg adg) {
    List<String> route = adg.routeOf(vehicle);
    int coordinatedStep = adg.coordinatedStepOf(vehicle);
    if (route.isEmpty() || coordinatedStep == Integer.MAX_VALUE) {
      return false;
    }
    int provenRouteEnd = Math.max(0, Math.min(route.size(), coordinatedStep + 1));
    List<String> provenRoute = route.subList(0, provenRouteEnd);
    for (TCSResource<?> resource : resources) {
      if (resource instanceof Point && !provenRoute.contains(resource.getName())) {
        return true;
      }
    }
    return false;
  }

  private boolean planIsTooOldToTrust(Adg adg) {
    return adg.builtAtMs() == 0L
        || millisClock.getAsLong() - adg.builtAtMs() > MAX_PLAN_AGE_MS;
  }

  private Step firstBlockerStillOnItsStep(
      Adg adg, Step wanted, Map<String, String> positions) {
    for (Step blocker : adg.predecessorsOf(wanted)) {
      if (blocker.vehicle().equals(wanted.vehicle()) || !hasOpenTcsOrder(blocker.vehicle())) {
        continue;
      }
      if (nextStepOf(blocker.vehicle(), adg, positions) - 1 <= blocker.step()) {
        return blocker;
      }
    }
    return null;
  }

  private static List<String> pointsIn(Set<TCSResource<?>> resources) {
    List<String> points = new ArrayList<>();
    for (TCSResource<?> resource : resources) {
      if (resource instanceof Point) {
        points.add(resource.getName());
      }
    }
    return points;
  }

  private static boolean requests(Set<TCSResource<?>> resources, String point) {
    for (TCSResource<?> resource : resources) {
      if (resource instanceof Point && resource.getName().equals(point)) {
        return true;
      }
    }
    return false;
  }

  private boolean requestLeftThePlan(
      String vehicle, Optional<PlanProgress> progress, Set<TCSResource<?>> resources, Adg adg) {
    if (!openTcsStillAllocatesAPlannedRoute(vehicle, adg) || planIsTooOldToTrust(adg)) {
      return false;
    }
    return progress.isEmpty() || progress.get().hasAlreadyPassedAnyOf(pointsIn(resources));
  }

  private boolean openTcsStillAllocatesAPlannedRoute(String vehicle, Adg adg) {
    List<String> claim = claims.get(vehicle);
    return !adg.routeOf(vehicle).isEmpty() && claim != null && !claim.isEmpty();
  }

  private Optional<PlanProgress> progressOf(String vehicle, Adg adg) {
    List<String> claim = claims.get(vehicle);
    return claim == null ? Optional.empty() : PlanProgress.of(adg.routeOf(vehicle), claim);
  }

  private int nextStepOf(String vehicle, Adg adg, Map<String, String> positions) {
    List<String> route = adg.routeOf(vehicle);
    if (route.isEmpty()) {
      return Integer.MAX_VALUE;
    }
    int standingAt = scanAheadFromLastKnownStep(vehicle, route, positions.get(vehicle));
    if (standingAt != OFF_PLAN) {
      lastKnownStep.put(vehicle, standingAt);
      return standingAt + 1;
    }
    vetoLog.fellBackToClaims();
    return progressOf(vehicle, adg).map(PlanProgress::step).orElse(0);
  }

  private int scanAheadFromLastKnownStep(String vehicle, List<String> route, String position) {
    if (position == null) {
      return OFF_PLAN;
    }
    int from = Math.min(lastKnownStep.getOrDefault(vehicle, 0), route.size());
    int ahead = route.subList(from, route.size()).indexOf(position);
    return ahead < 0 ? OFF_PLAN : from + ahead;
  }

  private boolean hasOpenTcsOrder(String vehicle) {
    return plantModelService.fetch(Vehicle.class, vehicle)
        .map(Vehicle::isProcessingOrder)
        .orElse(false);
  }

  private Map<String, String> currentPositions() {
    Map<String, String> positions = new HashMap<>();
    for (Vehicle candidate : plantModelService.fetch(Vehicle.class)) {
      TCSObjectReference<Point> position = candidate.getCurrentPosition();
      if (position != null) {
        positions.put(candidate.getName(), position.getName());
      }
    }
    return positions;
  }

  private static String vehicleOf(Scheduler.Client client) {
    TCSObjectReference<Vehicle> ref = client.getRelatedVehicle();
    return ref == null ? null : ref.getName();
  }
}
