package org.opentcs.fms.kernel;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.SynchronousQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import org.opentcs.components.kernel.Scheduler;
import org.opentcs.components.kernel.services.InternalVehicleService;
import org.opentcs.customizations.kernel.KernelExecutor;
import org.opentcs.data.order.DriveOrder;
import org.opentcs.data.order.ReroutingType;
import org.opentcs.data.order.Route;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.data.model.Vehicle;
import org.opentcs.fms.kernel.mapf.ClaimPolicy;
import org.opentcs.fms.kernel.mapf.MapfCoordinator;
import org.opentcs.fms.kernel.mapf.MapfPlanner;
import org.opentcs.fms.kernel.vda5050.VehicleErrors;
import org.opentcs.strategies.basic.dispatching.DefaultDispatcher;
import org.opentcs.strategies.basic.dispatching.DefaultDispatcherConfiguration;
import org.opentcs.strategies.basic.dispatching.FullDispatchTask;
import org.opentcs.strategies.basic.dispatching.OrderReservationPool;
import org.opentcs.strategies.basic.dispatching.PeriodicVehicleRedispatchingTask;
import org.opentcs.strategies.basic.dispatching.RerouteUtil;
import org.opentcs.strategies.basic.dispatching.TransportOrderAssignmentChecker;
import org.opentcs.strategies.basic.dispatching.TransportOrderUtil;
import org.opentcs.strategies.basic.dispatching.phase.assignment.OrderAssigner;
import org.opentcs.strategies.basic.routing.DefaultRouter;

import jakarta.inject.Inject;
import jakarta.inject.Named;
import jakarta.inject.Provider;

public class FMSDispatcher extends DefaultDispatcher {

  DefaultRouter router;
  private static final org.slf4j.Logger logger = org.slf4j.LoggerFactory.getLogger(FMSDispatcher.class);

  private static final long HORIZON_TICK_MS = 1_000L;

  private static final long SOLVE_WATCHDOG_MS = 10_000L;

  private final InternalVehicleService vehicleService;
  private final MapfCoordinator coordinator;
  private final RerouteUtil rerouteUtil;
  private final FleetSnapshotService fleetSnapshot;
  private final ScheduledExecutorService kernelExecutor;
  private final boolean mapfEnabled;
  private ExecutorService solver;
  private ScheduledFuture<?> horizonTickFuture;
  private MapfCoordinator.SolveRequest inFlight;
  private long solveStartedAtNanos;
  private Scheduler scheduler;

  @Inject
  public FMSDispatcher(OrderReservationPool orderReservationPool, TransportOrderUtil transportOrderUtil,
      InternalVehicleService vehicleService,
      @KernelExecutor ScheduledExecutorService kernelExecutor,
      FullDispatchTask fullDispatchTask, Provider<PeriodicVehicleRedispatchingTask> periodicDispatchTaskProvider,
      DefaultDispatcherConfiguration configuration, RerouteUtil rerouteUtil, OrderAssigner orderAssigner,
      TransportOrderAssignmentChecker transportOrderAssignmentChecker, MapfCoordinator coordinator,
      FleetSnapshotService fleetSnapshot,
      @Named(FmsKernelConfiguration.MAPF_ENABLED) boolean mapfEnabled, Scheduler scheduler) {
    super(orderReservationPool, transportOrderUtil, vehicleService, kernelExecutor, fullDispatchTask,
        periodicDispatchTaskProvider, configuration, rerouteUtil, orderAssigner,
        transportOrderAssignmentChecker);
    this.vehicleService = vehicleService;
    this.coordinator = coordinator;
    this.rerouteUtil = rerouteUtil;
    this.fleetSnapshot = fleetSnapshot;
    this.kernelExecutor = kernelExecutor;
    this.mapfEnabled = mapfEnabled;
    this.scheduler = scheduler;
  }

  @Override
  public void initialize() {
    if (isInitialized()) {
      return;
    }
    super.initialize();
    if (!mapfEnabled) {
      logger.info("[MAPF] disabled by configuration; no solves, no horizon tick");
      return;
    }
    solver = new ThreadPoolExecutor(
        1, 1, 0L, TimeUnit.MILLISECONDS, new SynchronousQueue<>(),
        Thread.ofPlatform().name("mapf-solver").daemon().factory());
    horizonTickFuture = kernelExecutor.scheduleAtFixedRate(
        this::horizonTick, HORIZON_TICK_MS, HORIZON_TICK_MS, TimeUnit.MILLISECONDS);
  }

  @Override
  public void terminate() {
    if (!isInitialized()) {
      return;
    }
    if (horizonTickFuture != null) {
      horizonTickFuture.cancel(false);
      horizonTickFuture = null;
    }
    if (solver != null) {
      solver.shutdownNow();
      solver = null;
    }
    if (inFlight != null) {
      inFlight = null;
      coordinator.abortSolve("the dispatcher is terminating");
    }
    super.terminate();
  }

  private void horizonTick() {
    failStuckSolve();
    try {
      dispatch();
    } catch (Throwable t) {
      logger.warn("[MAPF horizon] dispatch failed this tick: {}", t.toString());
    }
  }

  @Override
  public void dispatch() {
    submitSolve();
    super.dispatch();
  }

  private void submitSolve() {
    ExecutorService worker = solver;
    if (worker == null || inFlight != null) {
      return;
    }
    MapfCoordinator.SolveRequest request;
    try {
      FleetSnapshotService.Fleet fleet = fleetSnapshot.read();
      request = coordinator.prepare(
          fleet.positions(), fleet.goals(), fleet.immobile(), fleet.claimPolicy(),
          fleet.bodyState(), fleet.committedPrefixes());
    } catch (Throwable t) {
      logger.warn("[MAPF] coordination failed this cycle (assignment continues): {}", t.toString());
      return;
    }
    if (request == null) {
      return;
    }
    inFlight = request;
    solveStartedAtNanos = System.nanoTime();
    try {
      worker.execute(() -> solveOffKernelExecutor(request));
    } catch (RejectedExecutionException e) {
      inFlight = null;
      coordinator.abortSolve("the solver executor refused the task: " + e);
    }
  }

  private void solveOffKernelExecutor(MapfCoordinator.SolveRequest request) {
    MapfPlanner.PlanResult result;
    try {
      result = coordinator.compute(request);
    } catch (Throwable t) {
      kernelExecutor.execute(() -> abandonSolve(request, t));
      return;
    }
    kernelExecutor.execute(() -> installSolvedPlan(request, result));
  }

  private void abandonSolve(MapfCoordinator.SolveRequest request, Throwable cause) {
    if (inFlight != request) {
      return;
    }
    inFlight = null;
    coordinator.abortSolve("the solve threw " + cause);
  }

  private void installSolvedPlan(
      MapfCoordinator.SolveRequest request, MapfPlanner.PlanResult result) {
    if (inFlight != request) {
      logger.warn(
          "[MAPF solve] a solve the watchdog had already given up on finished; dropping its plan"
              + " so it cannot overwrite a newer one");
      return;
    }
    inFlight = null;
    long tookMs = (System.nanoTime() - solveStartedAtNanos) / 1_000_000L;
    boolean replanned;
    try {
      replanned = coordinator.install(request, result);
    } catch (Throwable t) {
      logger.warn("[MAPF] installing the plan failed this cycle: {}", t.toString());
      return;
    }
    logger.info("[MAPF solve] {}ms spent OFF the kernel executor, installed={}", tookMs, replanned);
    if (replanned) {
      rerouteChangedVehicles();
      this.scheduler.reschedule();
    }
  }

  private void failStuckSolve() {
    MapfCoordinator.SolveRequest stuck = inFlight;
    if (stuck == null) {
      return;
    }
    long heldMs = (System.nanoTime() - solveStartedAtNanos) / 1_000_000L;
    if (heldMs < SOLVE_WATCHDOG_MS) {
      return;
    }
    inFlight = null;
    coordinator.abortSolve(
        "a solve has held the slot for " + heldMs + "ms; forcing it open, otherwise the fleet"
            + " would never replan again");
  }

  private void rerouteChangedVehicles() {
    int rerouted = 0;
    int kept = 0;
    for (Vehicle vehicle : vehicleService.fetch(Vehicle.class)) {
      if (vehicle.getCurrentPosition() == null || !vehicle.isProcessingOrder()) {
        continue;
      }
      if (VehicleErrors.isOnlyLostNavigation(vehicle)) {
        continue;
      }
      if (!routeDiverges(vehicle)) {
        kept++;
        continue;
      }
      try {
        rerouteUtil.reroute(vehicle, ReroutingType.REGULAR);
        rerouted++;
      } catch (Throwable t) {
        logger.warn("[MAPF] reroute of {} failed this cycle: {}", vehicle.getName(), t.toString());
      }
    }
    logger.info("[MAPF reroute] {} rerouted, {} kept (route unchanged)", rerouted, kept);
  }

  private boolean routeDiverges(Vehicle vehicle) {
    Optional<TransportOrder> order = fleetSnapshot.currentOrder(vehicle);
    if (order.isEmpty()) {
      return true;
    }
    Optional<String> goal = fleetSnapshot.goalOf(order.get());
    if (goal.isEmpty()) {
      return true;
    }
    String from = fleetSnapshot.frontierPosition(vehicle);
    if (from.equals(goal.get())) {
      return false;
    }
    ClaimPolicy policy = ClaimPolicies.of(List.of(vehicle), ignored -> order);
    Optional<List<String>> served = coordinator.pointsFor(
        vehicle.getName(), from, goal.get(), policy);
    if (served.isEmpty()) {
      return false;
    }
    List<String> committed = committedSuffix(order.get(), from);
    if (committed.isEmpty()) {
      return true;
    }
    return !served.get().equals(committed);
  }

  private List<String> committedSuffix(TransportOrder order, String from) {
    DriveOrder current = order.getCurrentDriveOrder();
    if (current == null || current.getRoute() == null) {
      return List.of();
    }
    List<String> points = new ArrayList<>();
    for (Route.Step step : current.getRoute().getSteps()) {
      if (points.isEmpty() && step.getSourcePoint() != null) {
        points.add(step.getSourcePoint().getName());
      }
      if (step.getDestinationPoint() != null) {
        points.add(step.getDestinationPoint().getName());
      }
    }
    return suffixFrom(points, from, order.getCurrentRouteStepIndex() + 1);
  }

  static List<String> suffixFrom(List<String> route, String from, int minIndex) {
    for (int i = Math.max(minIndex, 0); i < route.size(); i++) {
      if (route.get(i).equals(from)) {
        return route.subList(i, route.size());
      }
    }
    return List.of();
  }
}
