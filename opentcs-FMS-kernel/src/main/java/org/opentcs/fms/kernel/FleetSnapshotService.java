package org.opentcs.fms.kernel;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Collectors;

import org.opentcs.components.kernel.services.InternalTCSObjectService;
import org.opentcs.components.kernel.services.InternalVehicleService;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.DriveOrder;
import org.opentcs.data.order.Route;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.fms.kernel.mapf.BodyState;
import org.opentcs.fms.kernel.mapf.ClaimPolicy;
import org.opentcs.fms.kernel.mapf.GoalResolver;
import org.opentcs.fms.kernel.rerouting.CommittedRouteFrontier;
import org.opentcs.fms.kernel.vda5050.VehicleErrors;
import org.opentcs.strategies.basic.dispatching.priorization.CompositeOrderComparator;
import org.opentcs.strategies.basic.dispatching.rerouting.VehiclePositionResolver;
import org.opentcs.strategies.basic.dispatching.selection.vehicles.IsAvailableForAnyOrder;

import jakarta.inject.Inject;

public class FleetSnapshotService {

  private static final org.slf4j.Logger logger = org.slf4j.LoggerFactory.getLogger(FleetSnapshotService.class);

  private String lastFleetSnapshot;
  private String lastUnlocalised;
  private String lastArrivals;
  private final InternalVehicleService vehicleService;
  private final InternalTCSObjectService objectService;
  private final VehiclePositionResolver vehiclePositionResolver;
  private final CompositeOrderComparator orderComparator;
  private final IsAvailableForAnyOrder availableForAnyOrder;
  private final CommittedRouteFrontier committedRouteFrontier;
  private final Map<String, String> prefixGapByVehicle = new HashMap<>();

  @Inject
  public FleetSnapshotService(
      InternalVehicleService vehicleService, InternalTCSObjectService objectService,
      VehiclePositionResolver vehiclePositionResolver, CompositeOrderComparator orderComparator,
      IsAvailableForAnyOrder availableForAnyOrder,
      CommittedRouteFrontier committedRouteFrontier) {
    this.vehicleService = vehicleService;
    this.objectService = objectService;
    this.vehiclePositionResolver = vehiclePositionResolver;
    this.orderComparator = orderComparator;
    this.availableForAnyOrder = availableForAnyOrder;
    this.committedRouteFrontier = committedRouteFrontier;
  }

  public record Fleet(
      Map<String, String> positions, Map<String, String> goals, Set<String> immobile,
      ClaimPolicy claimPolicy, BodyState bodyState,
      Map<String, List<String>> committedPrefixes) {
  }

  public Fleet read() {
    Map<String, List<String>> committedPrefixes = new HashMap<>();
    Map<String, String> positions = new HashMap<>();
    Map<String, String> goals = new HashMap<>();
    Set<String> immobile = new HashSet<>();
    List<String> heldByFlag = new ArrayList<>();
    List<String> faulted = new ArrayList<>();
    List<String> paused = new ArrayList<>();
    List<String> lostNavigation = new ArrayList<>();
    List<String> unlocalised = new ArrayList<>();
    List<String> movableWithoutOrder = new ArrayList<>();
    var fleetVehicles = vehicleService.fetch(Vehicle.class);
    Map<String, TransportOrder> pendingOrders = pendingOrdersByVehicle(fleetVehicles);
    Map<String, VehicleOrderContext> orderContexts = effectiveOrders(fleetVehicles, pendingOrders);
    for (Vehicle vehicle : fleetVehicles) {
      if (vehicle.getCurrentPosition() == null) {
        unlocalised.add(vehicle.getName());
        continue;
      }
      String name = vehicle.getName();
      boolean reportsLostNavigation = vehicle.hasState(Vehicle.State.ERROR)
          && VehicleErrors.isOnlyLostNavigation(vehicle);
      String position = vehicle.getCurrentPosition().getName();
      positions.put(name, position);
      List<String> inFlight = reportsLostNavigation
          ? List.of()
          : committedPrefix(vehicle, position);
      if (reportsLostNavigation) {
        prefixGapCleared(name);
      }
      if (!inFlight.isEmpty()) {
        committedPrefixes.put(name, inFlight);
      }

      if (vehicle.isPaused()) {
        paused.add(name);
        goals.put(name, position);
        immobile.add(name);
        continue;
      }
      if (vehicle.hasState(Vehicle.State.ERROR) && !reportsLostNavigation) {
        faulted.add(name);
        goals.put(name, position);
        immobile.add(name);
        continue;
      }
      if (waitsForDropOff(vehicle) && !vehicle.isProcessingOrder()
          && !pendingOrders.containsKey(name)) {
        heldByFlag.add(name);
        goals.put(name, position);
        immobile.add(name);
        continue;
      }
      if (reportsLostNavigation) {
        lostNavigation.add(name);
      }
      if (!vehicle.isProcessingOrder()) {
        movableWithoutOrder.add(name);
      }
      VehicleOrderContext orderContext = orderContexts.get(name);
      goals.put(name, orderContext == null
          ? position
          : orderContext.goal().orElse(position));
    }

    List<String> movers = new ArrayList<>();
    for (Map.Entry<String, String> e : goals.entrySet()) {
      if (!e.getValue().equals(positions.get(e.getKey()))) {
        movers.add(e.getKey() + "->" + e.getValue());
      }
    }
    movers.sort(null);
    heldByFlag.sort(null);
    faulted.sort(null);
    paused.sort(null);
    lostNavigation.sort(null);
    unlocalised.sort(null);
    movableWithoutOrder.sort(null);
    String snapshot = positions.size() + "|" + movers + "|" + immobile.size() + "|" + heldByFlag
        + "|" + faulted + "|" + paused + "|" + lostNavigation + "|" + movableWithoutOrder;
    if (!snapshot.equals(lastFleetSnapshot)) {
      lastFleetSnapshot = snapshot;
      logger.info(
          "[MAPF fleet] {} localised | {} with REAL goal {} | {} IMMOBILE (their points become"
              + " walls; see [MAPF mask] for which), waiting for drop-off: {}, faulted: {},"
              + " paused: {}"
              + " | {} lost navigation {} (planned from the position they REPORT, not the one"
              + " their last command aimed at)"
              + " | {} mobile agent(s) without an order {} (openTCS executes no move for these)",
          positions.size(), movers.size(), movers, immobile.size(),
          heldByFlag, faulted, paused, lostNavigation.size(), lostNavigation,
          movableWithoutOrder.size(), movableWithoutOrder);
    }
    if (!unlocalised.isEmpty() && !unlocalised.toString().equals(lastUnlocalised)) {
      lastUnlocalised = unlocalised.toString();
      logger.warn(
          "[MAPF fleet] {} vehicle(s) without a known position {}: absent from the problem"
              + " entirely, so the points they occupy are invisible and other vehicles may be"
              + " routed straight through them",
          unlocalised.size(), unlocalised);
    }
    Map<String, AgentAtGoal.Arrival> atGoal = AgentAtGoal.among(
        fleetVehicles, vehicle -> orderOf(vehicle, orderContexts), positions, goals,
        this::isRestingPoint);
    logArrivals(atGoal);
    ClaimPolicy claimPolicy = ClaimPolicies.of(
        fleetVehicles, vehicle -> orderOf(vehicle, orderContexts), atGoal.keySet());
    return new Fleet(positions, goals, immobile,
        claimPolicy,
        BodyStates.of(fleetVehicles, positions, this::currentOrder),
        Map.copyOf(committedPrefixes));
  }

  public Map<String, List<String>> installedRoutes() {
    Map<String, List<String>> routes = new HashMap<>();
    for (Vehicle vehicle : vehicleService.fetch(Vehicle.class)) {
      Optional<TransportOrder> order = currentOrder(vehicle);
      if (order.isEmpty()) {
        continue;
      }
      DriveOrder driveOrder = order.get().getCurrentDriveOrder();
      if (driveOrder == null || driveOrder.getRoute() == null) {
        continue;
      }
      routes.put(vehicle.getName(), List.copyOf(routePointsOf(driveOrder)));
    }
    return Map.copyOf(routes);
  }

  private Optional<TransportOrder> orderOf(
      Vehicle vehicle, Map<String, VehicleOrderContext> orderContexts) {
    VehicleOrderContext context = orderContexts.get(vehicle.getName());
    return context == null ? Optional.empty() : Optional.of(context.order());
  }

  private Map<String, VehicleOrderContext> effectiveOrders(
      Collection<Vehicle> fleetVehicles, Map<String, TransportOrder> pendingOrders) {
    Map<String, VehicleOrderContext> result = new HashMap<>();
    for (Vehicle vehicle : fleetVehicles) {
      Optional<TransportOrder> current = currentOrder(vehicle);
      if (current.isPresent()) {
        TransportOrder order = current.get();
        result.put(vehicle.getName(), new VehicleOrderContext(
            order, effectiveGoalOf(order), VehicleOrderContext.Source.CURRENT));
        continue;
      }
      TransportOrder pending = pendingOrders.get(vehicle.getName());
      if (pending != null) {
        result.put(vehicle.getName(), new VehicleOrderContext(
            pending, goalPointOf(pending), VehicleOrderContext.Source.PENDING));
      }
    }
    return result;
  }

  private Optional<String> effectiveGoalOf(TransportOrder order) {
    if (!stillHasADestination(order)) {
      return Optional.empty();
    }
    return goalOf(order).or(() -> goalPointOf(order));
  }

  private List<String> committedPrefix(Vehicle vehicle, String standingOn) {
    Optional<TransportOrder> order = currentOrder(vehicle);
    if (order.isEmpty()) {
      return noPrefix(vehicle, standingOn, "it has no transport order");
    }
    DriveOrder driveOrder = order.get().getCurrentDriveOrder();
    if (driveOrder == null || driveOrder.getRoute() == null) {
      return noPrefix(vehicle, standingOn, "its order carries no route");
    }
    List<String> points = routePointsOf(driveOrder);
    int start = order.get().getCurrentRouteStepIndex() + 1;
    if (start < 0 || start >= points.size() || !points.get(start).equals(standingOn)) {
      return noPrefix(vehicle, standingOn,
          "openTCS puts it at route index " + start + " of " + points);
    }
    int frozen = committedRouteFrontier.frozenStepIndex(
        vehicle, order.get().getCurrentRouteStepIndex());
    int end = frozen + 1;
    if (end <= start) {
      prefixGapCleared(vehicle.getName());
      return List.of();
    }
    if (end >= points.size()) {
      return noPrefix(vehicle, standingOn,
          "frozen route step index " + frozen + " is beyond " + points);
    }
    Optional<String> commandFrontier = resolvedCommandFrontier(vehicle);
    if (commandFrontier.isPresent() && !points.get(end).equals(commandFrontier.get())) {
      return noPrefix(vehicle, standingOn,
          "frozen route step index " + frozen + " ends at " + points.get(end)
              + " but the last command ends at " + commandFrontier.get());
    }
    prefixGapCleared(vehicle.getName());
    return List.copyOf(points.subList(start, end + 1));
  }

  private static List<String> routePointsOf(DriveOrder driveOrder) {
    List<String> points = new ArrayList<>();
    for (Route.Step step : driveOrder.getRoute().getSteps()) {
      if (points.isEmpty() && step.getSourcePoint() != null) {
        points.add(step.getSourcePoint().getName());
      }
      if (step.getDestinationPoint() != null) {
        points.add(step.getDestinationPoint().getName());
      }
    }
    return points;
  }

  private List<String> noPrefix(Vehicle vehicle, String standingOn, String reason) {
    if (!reason.equals(prefixGapByVehicle.put(vehicle.getName(), reason))) {
      logger.info(
          "[MAPF prefix-gap] {} stands on {}: no committed prefix because {}. The solver starts"
              + " at the real position, but cannot reserve the in-flight edge.",
          vehicle.getName(), standingOn, reason);
    }
    return List.of();
  }

  private void prefixGapCleared(String vehicle) {
    prefixGapByVehicle.remove(vehicle);
  }

  private Optional<String> resolvedCommandFrontier(Vehicle vehicle) {
    try {
      return Optional.ofNullable(vehiclePositionResolver.getFutureOrCurrentPosition(vehicle))
          .map(Point::getName);
    } catch (RuntimeException e) {
      return Optional.empty();
    }
  }

  public String frontierPosition(Vehicle vehicle) {
    try {
      Point frontier = vehiclePositionResolver.getFutureOrCurrentPosition(vehicle);
      if (frontier != null) {
        return frontier.getName();
      }
    } catch (RuntimeException e) {
      logger.debug("[MAPF] frontier resolve failed for {}, using current position: {}",
          vehicle.getName(), e.toString());
    }
    return vehicle.getCurrentPosition().getName();
  }

  private void logArrivals(Map<String, AgentAtGoal.Arrival> atGoal) {
    String arrivals = atGoal.toString();
    if (arrivals.equals(lastArrivals)) {
      return;
    }
    lastArrivals = arrivals;
    logger.info(
        "[MAPF at-goal] {} vehicle(s) count as having reached a real goal: {}. Vehicles between"
            + " two transport orders of one task are excluded.",
        atGoal.size(), arrivals);
  }

  private boolean isRestingPoint(String point) {
    return objectService.fetch(Point.class, point)
        .map(candidate -> candidate.getType() == Point.Type.PARK_POSITION)
        .orElse(false);
  }

  public Optional<TransportOrder> currentOrder(Vehicle vehicle) {
    if (vehicle.getTransportOrder() == null) {
      return Optional.empty();
    }
    return vehicleService.fetch(TransportOrder.class, vehicle.getTransportOrder().getName());
  }

  public Optional<String> goalOf(TransportOrder order) {
    if (!stillHasADestination(order)) {
      return Optional.empty();
    }
    DriveOrder current = order.getCurrentDriveOrder();
    if (current == null || current.getRoute() == null) {
      return Optional.empty();
    }
    List<Route.Step> steps = current.getRoute().getSteps();
    if (steps.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(steps.get(steps.size() - 1).getDestinationPoint().getName());
  }

  private static boolean waitsForDropOff(Vehicle vehicle) {
    return Boolean.parseBoolean(
        vehicle.getProperty(WaitDropOffListener.WAIT_DROPOFF_PROPERTY));
  }

  private Map<String, TransportOrder> pendingOrdersByVehicle(Collection<Vehicle> fleetVehicles) {
    Map<String, Vehicle> vehiclesByName = fleetVehicles.stream()
        .collect(Collectors.toMap(Vehicle::getName, Function.identity()));

    Map<String, TransportOrder> claimedByVehicle = new HashMap<>();
    for (TransportOrder order : vehicleService.fetch(TransportOrder.class)) {
      if (order.getIntendedVehicle() == null) {
        continue;
      }
      String vehicleName = order.getIntendedVehicle().getName();
      if (!aboutToBeDriven(order, vehiclesByName.get(vehicleName))) {
        continue;
      }
      claimedByVehicle.merge(
          vehicleName, order, (kept, other) -> orderComparator.compare(kept, other) <= 0
              ? kept
              : other);
    }

    return claimedByVehicle;
  }

  private boolean aboutToBeDriven(TransportOrder order, Vehicle intendedVehicle) {
    if (!stillHasADestination(order)) {
      return false;
    }
    return order.getState() == TransportOrder.State.BEING_PROCESSED
        || (intendedVehicle != null && availableForAnyOrder.test(intendedVehicle));
  }

  private static boolean stillHasADestination(TransportOrder order) {
    return switch (order.getState()) {
      case RAW, ACTIVE, DISPATCHABLE, BEING_PROCESSED -> true;
      case WITHDRAWN, FINISHED, FAILED, UNROUTABLE -> false;
    };
  }

  private Optional<String> goalPointOf(TransportOrder order) {
    DriveOrder driveOrder = order.getCurrentDriveOrder();
    if (driveOrder == null) {
      List<DriveOrder> all = order.getAllDriveOrders();
      if (all.isEmpty()) {
        return Optional.empty();
      }
      driveOrder = all.get(0);
    }
    return GoalResolver.goalPoint(driveOrder.getDestination(), objectService);
  }
}
