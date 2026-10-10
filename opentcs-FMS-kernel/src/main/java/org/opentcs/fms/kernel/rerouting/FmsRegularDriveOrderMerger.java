package org.opentcs.fms.kernel.rerouting;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.ReroutingType;
import org.opentcs.data.order.Route;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.fms.kernel.mapf.MapfCaptureStore;
import org.opentcs.drivers.vehicle.VehicleControllerPool;
import org.opentcs.strategies.basic.dispatching.rerouting.RegularDriveOrderMerger;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import jakarta.inject.Inject;

public class FmsRegularDriveOrderMerger
    extends
    RegularDriveOrderMerger {

  private static final Logger LOG = LoggerFactory.getLogger(FmsRegularDriveOrderMerger.class);
  private static final Set<SpliceRevisitEpisode> LOGGED_SPLICE_REVISITS =
      ConcurrentHashMap.newKeySet();

  private record SpliceRevisitEpisode(String vehicle, List<String> route) {
  }

  private record SpliceDiagnostics(
      List<String> oldRoute, List<String> keptPrefix, List<String> sentPrefix,
      List<String> newRoute, List<String> mergedRoute, List<String> executingRoute,
      String splicePoint,
      String planFrom, String sentEnd) {
  }

  private final CommittedRouteFrontier committedRouteFrontier;

  @Inject
  public FmsRegularDriveOrderMerger(VehicleControllerPool vehicleControllerPool) {
    this.committedRouteFrontier = new CommittedRouteFrontier(vehicleControllerPool);
  }

  @Override
  protected Route mergeRoutes(
      Route routeA,
      Route routeB,
      TransportOrder originalOrder,
      int currentRouteStepIndex,
      Vehicle vehicle) {
    int frozenUntilIndex = committedRouteFrontier.frozenStepIndex(vehicle, currentRouteStepIndex);
    List<Route.Step> oldSteps = routeA.getSteps();
    List<Route.Step> newSteps = routeB.getSteps();
    int spliceIndex = divergingStepIndex(newSteps.get(0).getSourcePoint(), oldSteps,
        frozenUntilIndex);
    List<Route.Step> merged = mergeSteps(oldSteps, newSteps, frozenUntilIndex);
    if (spliceIndex > 0) {
      List<String> solverCommittedPrefix = MapfCaptureStore
          .takeRouteCommitment(routeB, vehicle.getName())
          .map(MapfCaptureStore.RouteCommitment::solverCommittedPrefix)
          .orElse(List.of());
      SpliceDiagnostics diagnostics = spliceDiagnostics(
          vehicle, oldSteps, newSteps, merged, currentRouteStepIndex, frozenUntilIndex,
          spliceIndex);
      if (!diagnostics.keptPrefix().isEmpty()) {
        logSplice(vehicle, diagnostics, solverCommittedPrefix, spliceIndex);
        logSpliceRevisit(vehicle, diagnostics);
      }
    }
    return new Route(merged);
  }

  private SpliceDiagnostics spliceDiagnostics(
      Vehicle vehicle, List<Route.Step> oldSteps, List<Route.Step> newSteps,
      List<Route.Step> mergedSteps, int currentRouteStepIndex, int frozenUntilIndex,
      int spliceIndex) {
    List<String> oldRoute = pointNames(oldSteps);
    List<String> newRoute = pointNames(newSteps);
    int start = Math.min(Math.max(currentRouteStepIndex + 1, 0), oldRoute.size());
    int spliceEnd = Math.min(spliceIndex + 1, oldRoute.size());
    List<String> keptPrefix = oldRoute.subList(start, spliceEnd);
    List<String> sentPrefix = oldRoute.subList(
        start, Math.min(Math.max(frozenUntilIndex + 1, 0), spliceEnd));
    String splicePoint = spliceIndex < oldRoute.size() ? oldRoute.get(spliceIndex) : "unknown";
    List<String> mergedRoute = pointNames(mergedSteps);
    List<String> executingRoute = routeFromCurrentPoint(
        mergedRoute, vehicle, currentRouteStepIndex);
    String planFrom = newRoute.isEmpty() ? "unknown" : newRoute.get(0);
    String sentEnd = sentPrefix.isEmpty()
        ? currentPoint(oldRoute, vehicle, currentRouteStepIndex)
        : sentPrefix.get(sentPrefix.size() - 1);
    return new SpliceDiagnostics(
        oldRoute, keptPrefix, sentPrefix, newRoute, mergedRoute, executingRoute, splicePoint,
        planFrom, sentEnd);
  }

  private void logSplice(
      Vehicle vehicle, SpliceDiagnostics diagnostics, List<String> solverCommittedPrefix,
      int spliceIndex) {
    if (solverCommittedPrefix.size() < diagnostics.sentPrefix().size()) {
      LOG.warn("[MAPF prefix-mismatch] {} merger froze {} but the solver was given {}",
          vehicle.getName(), diagnostics.sentPrefix(), solverCommittedPrefix);
    }
    String line = "[MAPF splice] {} old {} kept {} (allocated/sent {}) + new {} splice {}"
        + " | planFrom={} sentEnd={} solverCommittedPrefix={}"
        + " | adg index offset +{} (kernel route index {})";
    boolean planBehindCommitment = planBehindCommitment(
        diagnostics.planFrom(), diagnostics.sentEnd());
    if (planBehindCommitment) {
      LOG.warn(line + " planBehindCommitment", vehicle.getName(), diagnostics.oldRoute(),
          diagnostics.keptPrefix(), diagnostics.sentPrefix(), diagnostics.newRoute(),
          diagnostics.splicePoint(), diagnostics.planFrom(), diagnostics.sentEnd(),
          solverCommittedPrefix, spliceIndex, spliceIndex);
      return;
    }
    LOG.info(line, vehicle.getName(), diagnostics.oldRoute(), diagnostics.keptPrefix(),
        diagnostics.sentPrefix(), diagnostics.newRoute(), diagnostics.splicePoint(),
        diagnostics.planFrom(), diagnostics.sentEnd(), solverCommittedPrefix,
        spliceIndex, spliceIndex);
  }

  private void logSpliceRevisit(Vehicle vehicle, SpliceDiagnostics diagnostics) {
    String revisitedPoint = firstRevisitedPointBeforeDestination(diagnostics.executingRoute());
    if (revisitedPoint == null || !LOGGED_SPLICE_REVISITS.add(
        new SpliceRevisitEpisode(vehicle.getName(), List.copyOf(diagnostics.mergedRoute())))) {
      return;
    }
    LOG.warn(
        "[MAPF splice-revisit] {} executes {} revisiting {} | kept {} sent {} + new {}",
        vehicle.getName(), diagnostics.executingRoute(), revisitedPoint,
        diagnostics.keptPrefix(), diagnostics.sentPrefix(), diagnostics.newRoute());
    MapfCaptureStore.dumpRecentIfAllowed("splice-revisit-" + vehicle.getName());
  }

  static boolean hasRevisitBeforeDestination(List<String> route) {
    return firstRevisitedPointBeforeDestination(route) != null;
  }

  static boolean planBehindCommitment(String planFrom, String sentEnd) {
    return !Objects.equals(planFrom, sentEnd);
  }

  private static String firstRevisitedPointBeforeDestination(List<String> route) {
    if (route.size() < 3) {
      return null;
    }
    Set<String> visited = new HashSet<>();
    String previous = null;
    int destinationIndex = route.size() - 1;
    for (int index = 0; index < route.size(); index++) {
      String point = route.get(index);
      if (Objects.equals(point, previous)) {
        continue;
      }
      if (index < destinationIndex && !visited.add(point)) {
        return point;
      }
      visited.add(point);
      previous = point;
    }
    return null;
  }

  private static List<String> routeFromCurrentPoint(
      List<String> route, Vehicle vehicle, int currentRouteStepIndex) {
    int start = Math.min(Math.max(currentRouteStepIndex + 1, 0), route.size());
    String current = vehicle.getCurrentPosition() == null
        ? null
        : vehicle.getCurrentPosition().getName();
    if (current == null || start == route.size() || current.equals(route.get(start))) {
      return route.subList(start, route.size());
    }
    List<String> executing = new ArrayList<>(route.size() - start + 1);
    executing.add(current);
    executing.addAll(route.subList(start, route.size()));
    return List.copyOf(executing);
  }

  private static String currentPoint(
      List<String> route, Vehicle vehicle, int currentRouteStepIndex) {
    if (vehicle.getCurrentPosition() != null) {
      return vehicle.getCurrentPosition().getName();
    }
    int index = Math.min(Math.max(currentRouteStepIndex + 1, 0), route.size() - 1);
    return route.isEmpty() ? "unknown" : route.get(index);
  }

  private static List<String> pointNames(List<Route.Step> steps) {
    List<String> points = new ArrayList<>();
    if (!steps.isEmpty() && steps.get(0).getSourcePoint() != null) {
      points.add(steps.get(0).getSourcePoint().getName());
    }
    for (Route.Step step : steps) {
      if (step.getDestinationPoint() != null) {
        points.add(step.getDestinationPoint().getName());
      }
    }
    return points;
  }

  @Override
  protected List<Route.Step> mergeSteps(
      List<Route.Step> stepsA,
      List<Route.Step> stepsB,
      int frozenUntilIndex) {
    List<Route.Step> mergedSteps = new ArrayList<>();

    int divergingIndex = divergingStepIndex(
        stepsB.get(0).getSourcePoint(), stepsA, frozenUntilIndex);
    mergedSteps.addAll(stepsA.subList(0, divergingIndex));

    List<Route.Step> modifiedStepsB = new ArrayList<>(stepsB);
    modifiedStepsB.set(0, stepsB.get(0).withReroutingType(ReroutingType.REGULAR));
    mergedSteps.addAll(modifiedStepsB);

    return updateRouteIndices(mergedSteps);
  }

  private int divergingStepIndex(
      Point sourcePoint, List<Route.Step> steps, int frozenUntilIndex) {
    for (int i = frozenUntilIndex + 1; i < steps.size(); i++) {
      if (Objects.equals(steps.get(i).getSourcePoint(), sourcePoint)) {
        return i;
      }
    }
    throw new IllegalStateException(
        "No step with source point " + sourcePoint
            + " at or after the committed frontier (step index " + frozenUntilIndex + ")");
  }

}
