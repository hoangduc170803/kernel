package org.opentcs.fms.kernel;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;

import org.opentcs.components.kernel.Router;
import org.opentcs.components.kernel.services.InternalTCSObjectService;
import org.opentcs.data.TCSObjectReference;
import org.opentcs.data.model.Path;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.TCSResourceReference;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.DriveOrder;
import org.opentcs.data.order.Route;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.fms.kernel.mapf.ClaimPolicy;
import org.opentcs.fms.kernel.mapf.GoalResolver;
import org.opentcs.fms.kernel.mapf.MapfCaptureStore;
import org.opentcs.fms.kernel.mapf.MapfCoordinator;
import org.opentcs.fms.kernel.mapf.MapfCoordinator.DriveOrderLeg;
import org.opentcs.fms.kernel.mapf.RoutingContext;
import org.opentcs.fms.kernel.scheduling.RetreatHolds;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.pibt.Footprint;
import org.opentcs.strategies.basic.routing.DefaultRouter;

import jakarta.inject.Inject;

public class FMSRouter
    implements
    Router {

  private static final org.slf4j.Logger logger = org.slf4j.LoggerFactory.getLogger(FMSRouter.class);

  static final String PROPKEY_UNROUTABLE_REASON = "fms:unroutableReason";

  private final DefaultRouter delegate;
  private final InternalTCSObjectService objectService;
  private final MapfCoordinator coordinator;

  private volatile boolean initialized;
  private volatile GoalGeometry geometry;

  @Inject
  public FMSRouter(DefaultRouter delegate, InternalTCSObjectService objectService,
      MapfCoordinator coordinator) {
    this.delegate = delegate;
    this.objectService = objectService;
    this.coordinator = coordinator;
  }

  @Override
  public void initialize() {
    if (initialized) {
      return;
    }
    delegate.initialize();
    coordinator.invalidateModel();
    initialized = true;
  }

  @Override
  public boolean isInitialized() {
    return initialized;
  }

  @Override
  public void terminate() {
    if (!initialized) {
      return;
    }
    delegate.terminate();
    initialized = false;
  }

  @Override
  public void updateRoutingTopology(Set<Path> paths) {
    delegate.updateRoutingTopology(paths);
    coordinator.invalidateModel();
  }

  @Override
  public Set<Vehicle> checkRoutability(TransportOrder order) {
    return delegate.checkRoutability(order);
  }

  @Override
  public boolean checkGeneralRoutability(TransportOrder order) {
    if (!RetreatHolds.isDropOffLeg(order)) {
      return delegate.checkGeneralRoutability(order);
    }
    Optional<String> goal = currentGoalOf(order);
    if (goal.isEmpty()) {
      return delegate.checkGeneralRoutability(order);
    }

    Optional<Clash> clash = firstClashWith(goal.get(), order);
    if (clash.isEmpty()) {
      return delegate.checkGeneralRoutability(order);
    }

    TransportOrder claimant = clash.get().claimant();
    logger.warn(
        "[MAPF goal-claim] order {} aims its current leg at {}, which cannot hold a vehicle"
            + " alongside {} - the goal of order {}, created at {}. The later order is dismissed"
            + " as unroutable ({})",
        order.getName(), goal.get(), clash.get().claimedGoal(), claimant.getName(),
        claimant.getCreationTime(), clash.get().reason());
    objectService.updateObjectProperty(
        order.getReference(), PROPKEY_UNROUTABLE_REASON, clash.get().reason());
    return false;
  }

  private record Clash(TransportOrder claimant, String claimedGoal, String reason) {
  }

  private Optional<Clash> firstClashWith(String goal, TransportOrder order) {
    GoalGeometry geometry = goalGeometry();
    return objectService.stream(TransportOrder.class)
        .filter(other -> !other.getName().equals(order.getName()))
        .filter(FMSRouter::stillHeadedSomewhere)
        .filter(other -> outranks(other, order))
        .map(other -> currentGoalOf(other)
            .map(claimed -> clashBetween(geometry, order, goal, claimed, other))
            .orElse(null))
        .filter(Objects::nonNull)
        .flatMap(Optional::stream)
        .findFirst();
  }

  private Optional<Clash> clashBetween(
      GoalGeometry geometry, TransportOrder order, String goal, String claimed,
      TransportOrder claimant) {
    if (goal.equals(claimed)) {
      if (handsTheGoalOver(claimant, order)) {
        logger.debug(
            "[MAPF goal-claim] order {} keeps goal {}: the older claim on that cell is {}, the"
                + " approach order of the same vehicle {}",
            order.getName(), goal, claimant.getName(), vehicleOf(order));
        return Optional.empty();
      }
      return Optional.of(
          new Clash(claimant, claimed, "duplicateGoal:" + goal + ":" + claimant.getName()));
    }
    if (geometry == null || !bodiesCannotCoexist(geometry, goal, claimed)) {
      return Optional.empty();
    }
    return Optional.of(new Clash(claimant, claimed,
        "goalFootprint:" + goal + ":" + claimed + ":" + claimant.getName()));
  }

  private static boolean bodiesCannotCoexist(GoalGeometry geometry, String goal, String claimed) {
    Integer here = geometry.roadmap().idOf(goal);
    Integer there = geometry.roadmap().idOf(claimed);
    if (here == null || there == null) {
      return false;
    }
    int[] hereNoses = arrivalNoses(geometry.roadmap(), here);
    int[] thereNoses = arrivalNoses(geometry.roadmap(), there);
    if (hereNoses.length == 0 || thereNoses.length == 0) {
      return false;
    }
    for (int nose : hereNoses) {
      for (int otherNose : thereNoses) {
        if (!geometry.footprint().bodiesClash(here, nose, false, there, otherNose, false)) {
          return false;
        }
      }
    }
    return true;
  }

  private static int[] arrivalNoses(Roadmap roadmap, int vertex) {
    int[] found = new int[Roadmap.NOSES];
    int count = 0;
    for (int predecessor : roadmap.predecessorsOf(vertex)) {
      int nose = roadmap.requiredNose(predecessor, vertex);
      if (nose == Roadmap.DIR_NONE) {
        continue;
      }
      boolean seen = false;
      for (int i = 0; i < count; i++) {
        seen |= found[i] == nose;
      }
      if (!seen) {
        found[count++] = nose;
      }
    }
    return Arrays.copyOf(found, count);
  }

  private GoalGeometry goalGeometry() {
    if (!Footprint.enabled()) {
      return null;
    }
    RoutingContext ctx = coordinator.context();
    if (ctx == null) {
      return null;
    }
    GoalGeometry current = geometry;
    if (current != null && current.context() == ctx) {
      return current;
    }
    GoalGeometry built = new GoalGeometry(ctx, ctx.roadmap(), new Footprint(ctx.roadmap()));
    geometry = built;
    return built;
  }

  private record GoalGeometry(RoutingContext context, Roadmap roadmap, Footprint footprint) {
  }

  private Optional<String> currentGoalOf(TransportOrder order) {
    DriveOrder leg = order.getCurrentDriveOrder();
    if (leg == null) {
      List<DriveOrder> legs = order.getAllDriveOrders();
      if (legs.isEmpty()) {
        return Optional.empty();
      }
      leg = legs.get(0);
    }
    return GoalResolver.goalPoint(leg.getDestination(), objectService);
  }

  private static boolean stillHeadedSomewhere(TransportOrder order) {
    return !order.getState().isFinalState()
        && order.getState() != TransportOrder.State.WITHDRAWN;
  }

  private static boolean handsTheGoalOver(TransportOrder claimant, TransportOrder order) {
    return RetreatHolds.isApproachLeg(claimant)
        && RetreatHolds.isDropOffLeg(order)
        && sameVehicle(claimant, order);
  }

  private static boolean sameVehicle(TransportOrder first, TransportOrder second) {
    String vehicle = vehicleOf(first);
    return vehicle != null && vehicle.equals(vehicleOf(second));
  }

  private static String vehicleOf(TransportOrder order) {
    TCSObjectReference<Vehicle> driving = order.getProcessingVehicle() != null
        ? order.getProcessingVehicle()
        : order.getIntendedVehicle();
    return driving == null ? null : driving.getName();
  }

  private static boolean outranks(TransportOrder first, TransportOrder second) {
    int byCreation = first.getCreationTime().compareTo(second.getCreationTime());
    return byCreation != 0
        ? byCreation < 0
        : first.getName().compareTo(second.getName()) < 0;
  }

  @Override
  public Set<Route> getRoutes(Vehicle vehicle, Point sourcePoint, Point destinationPoint,
      Set<TCSResourceReference<?>> resourcesToAvoid, int maxRoutesPerDriveOrder) {
    return delegate.getRoutes(vehicle, sourcePoint, destinationPoint, resourcesToAvoid,
        maxRoutesPerDriveOrder);
  }

  @Override
  public Set<List<Route>> getRoutes(Vehicle vehicle, Point sourcePoint, TransportOrder order,
      int maxRoutesPerDriveOrder) {
    RoutingContext ctx = coordinator.context();
    if (ctx == null || sourcePoint == null) {
      return Set.of();
    }
    try {
      List<Route> chain = new ArrayList<>();
      String src = sourcePoint.getName();
      List<DriveOrder> driveOrders = order.getAllDriveOrders();
      ClaimPolicy policy = ClaimPolicies.of(List.of(vehicle), v -> Optional.of(order));
      for (int leg = 1; leg <= driveOrders.size(); leg++) {
        DriveOrder driveOrder = driveOrders.get(leg - 1);
        Optional<String> resolved = GoalResolver.goalPoint(driveOrder.getDestination(), objectService)
            .filter(ctx.roadmap()::has);
        if (resolved.isEmpty() || !ctx.roadmap().has(src)) {
          logger.warn(
              "[MAPF router] {} order {}: leg {} runs between points the roadmap does not have"
                  + " (src={}); no route can be served",
              vehicle.getName(), order.getName(), leg, src);
          return Set.of();
        }
        String goal = resolved.get();
        if (src.equals(goal)) {
          Optional<Route> stay = ctx.routeBuilder().buildStayRoute(src);
          if (stay.isEmpty()) {
            return Set.of();
          }
          chain.add(stay.get());
          continue;
        }
        Optional<MapfCoordinator.DriveRoute> plannedDrive = coordinator
            .pointsWithCommitmentToDrive(
                vehicle.getName(), src, goal, new DriveOrderLeg(leg, driveOrders.size()),
                policy);
        if (plannedDrive.isEmpty()) {
          return Set.of();
        }
        List<String> points = plannedDrive.get().points();
        if (!points.get(points.size() - 1).equals(goal)) {
          logger.error(
              "[MAPF router] {} {}->{}: coordinated path ends at {} instead of the goal;"
                  + " no route can be served",
              vehicle.getName(), src, goal, points.get(points.size() - 1));
          return Set.of();
        }

        Optional<Route> route = ctx.routeBuilder().buildRoute(points);
        if (route.isEmpty()) {
          logger.error(
              "[MAPF router] {} {}->{}: the coordinated path could not be turned into steps;"
                  + " no route can be served",
              vehicle.getName(), src, goal);
          return Set.of();
        }
        MapfCaptureStore.registerRouteCommitment(
            route.get(), vehicle.getName(), plannedDrive.get().solverCommittedPrefix());
        chain.add(route.get());
        src = goal;
      }
      return chain.isEmpty() ? Set.of() : Set.of(chain);
    } catch (RuntimeException e) {
      logger.error("[MAPF router] routing order {} failed; no route can be served: {}",
          order.getName(), e.toString());
      return Set.of();
    }
  }
}
