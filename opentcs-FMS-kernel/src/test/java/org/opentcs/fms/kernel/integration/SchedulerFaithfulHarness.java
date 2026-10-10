package org.opentcs.fms.kernel.integration;

import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.io.File;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.TreeMap;
import java.util.concurrent.AbstractExecutorService;
import java.util.concurrent.Callable;
import java.util.concurrent.CancellationException;
import java.util.concurrent.Delayed;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import org.opentcs.components.kernel.Scheduler;
import org.opentcs.components.kernel.services.InternalPlantModelService;
import org.opentcs.components.kernel.services.PlantModelService;
import org.opentcs.data.TCSObjectReference;
import org.opentcs.data.model.Path;
import org.opentcs.data.model.PlantModel;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.TCSResource;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.fms.kernel.mapf.BodyState;
import org.opentcs.fms.kernel.mapf.ClaimPolicy;
import org.opentcs.fms.kernel.mapf.MapfCoordinator;
import org.opentcs.fms.kernel.mapf.MapfPrecedenceModule;
import org.opentcs.fms.kernel.mapf.adg.Adg;
import org.opentcs.fms.kernel.mapf.adg.AdgService;
import org.opentcs.fms.kernel.mapf.adg.Step;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.pibt.Footprint;
import org.opentcs.fms.kernel.scheduling.FMSScheduler;
import org.opentcs.strategies.basic.scheduling.AllocationAdvisor;
import org.opentcs.strategies.basic.scheduling.DefaultScheduler;
import org.opentcs.strategies.basic.scheduling.ReservationPool;
import org.opentcs.util.event.SimpleEventBus;

/**
 * FAITHFUL fleet harness: drives the agents through the <em>real</em> openTCS
 * {@link DefaultScheduler} + {@link ReservationPool} +
 * {@link AllocationAdvisor}
 * (the exact classes from opentcs-strategies-default 7.3.0), instead of any
 * hand-rolled occupancy/denial emulation.
 *
 * <p>
 * Why: the emulated harness re-solves from clean, synchronised integer
 * positions and applies new routes with no claim protocol, so it never
 * reproduces the kernel's high-load deadlock -- the {@code AllocatorTask}
 * "Not allocating resources that are not next claimed resources" storm that
 * arises when {@code rerouteAll} keeps replacing the claim faster than a
 * contended vehicle can allocate its next step. This harness plugs in the real
 * scheduler so that mechanism is present.
 *
 * <p>
 * What is real: the whole allocation path -- {@code claim} (a per-vehicle
 * queue of {@code {Path, Point}} resource sets), {@code allocate} (must match
 * the head of the claim, else deferred), collision-freedom via the reservation
 * pool, deferred-allocation retries on every {@code free}, and the
 * {@link MapfPrecedenceModule} ADG consulted by the scheduler itself. A minimal
 * per-vehicle driver mirrors the five pieces of
 * {@code DefaultVehicleController}
 * that matter (currentClaim / getNeededResources / allocate-next / free-behind
 * /
 * re-claim-on-reroute) -- no comm adapter, no transport-order service.
 *
 * <p>
 * Approximation: a same-thread executor runs {@code AllocatorTask}s inline
 * (deterministic), so the sync-check-vs-async-exec race is collapsed; the
 * churn-replaces-the-claim mechanism that actually causes the deadlock does not
 * depend on that race.
 */
final class SchedulerFaithfulHarness {

  private static final int STALL_LIMIT = 25;

  private static final List<String> PARK_POINTS = List.of(
      "0170", "0171", "0172", "0173", "0174", "0175", "0176", "0177", "0178", "0179",
      "0047", "0048", "0240", "0250", "0260", "0270", "1010", "1011", "1012",
      "0200", "0210", "0220", "0230",
      "1001", "1002", "1003", "1004", "1005", "1006",
      "1000", "1007", "1008", "1009");

  private static final int HEARTBEAT_TICKS = 1;
  private static final int LOG_EVERY = 1;

  private final PlantModel plantModel;
  private final MapfCoordinator coordinator;
  private final FMSScheduler scheduler;
  private final SameThreadScheduledExecutor exec = new SameThreadScheduledExecutor();

  private final Map<String, Path> pathByDir = new HashMap<>();
  private final Map<String, Point> pointByName = new HashMap<>();
  private final Map<String, Driver> vehicles = new TreeMap<>();
  private final Map<String, Integer> maxPlannedWhileWaiting = new HashMap<>();

  private boolean churn = true;
  private int reclaims;
  private int rejectedReroutes;
  private boolean enforceContinuity = false;
  private final StringBuilder journal = new StringBuilder();

  SchedulerFaithfulHarness(File mapXml) throws Exception {
    this.plantModel = PlantModelXmlLoader.load(mapXml);
    for (Point p : plantModel.getPoints()) {
      pointByName.put(p.getName(), p);
    }
    for (Path p : plantModel.getPaths()) {
      pathByDir.put(
          p.getSourcePoint().getName() + ">" + p.getDestinationPoint().getName(), p);
    }

    PlantModelService pms = mock(PlantModelService.class);
    when(pms.getPlantModel()).thenReturn(plantModel);
    AdgService adgService = new AdgService();
    this.coordinator = new MapfCoordinator(pms, adgService);
    this.coordinator.setClock(() -> tickClock * 700L);

    InternalPlantModelService ipms = mock(InternalPlantModelService.class);
    when(ipms.fetch(eq(Vehicle.class), anyString())).thenAnswer(
        inv -> Optional.of(vehicleUnderOpenTcsOrder(inv.getArgument(1))));

    Object globalSync = new Object();
    ReservationPool pool = new ReservationPool();

    MapfPrecedenceModule precedence = new MapfPrecedenceModule(adgService, ipms);
    precedence.setClock(() -> tickClock * 700L * 1_000_000L);
    precedence.setMillisClock(() -> tickClock * 700L);

    AllocationAdvisor advisor = new AllocationAdvisor(Set.<Scheduler.Module>of(precedence));

    this.scheduler = new FMSScheduler(
        advisor, pool, exec, new SimpleEventBus(), globalSync);
    scheduler.initialize();
  }

  private long tickClock;
  private long nextClaimRank = 1L;
  private final Set<String> carryingLoad = new HashSet<>();
  private volatile BodyState lastBodyState = BodyState.UNKNOWN;
  private final Map<String, String> firstPlannedGoal = new HashMap<>();

  String firstPlannedGoalOf(String vehicle) {
    return firstPlannedGoal.get(vehicle);
  }

  void carryLoad(String vehicle) {
    carryingLoad.add(vehicle);
  }

  BodyState lastBodyState() {
    return lastBodyState;
  }

  private BodyState bodyState() {
    Map<String, Integer> axisByVehicle = new HashMap<>();
    for (Driver d : vehicles.values()) {
      if (d.idx <= 0) {
        continue;
      }
      Point from = pointByName.get(d.route.get(d.idx - 1));
      Point to = pointByName.get(d.current());
      if (from == null || to == null) {
        continue;
      }
      long dx = Math.abs(to.getPose().getPosition().getX() - from.getPose().getPosition().getX());
      long dy = Math.abs(to.getPose().getPosition().getY() - from.getPose().getPosition().getY());
      if (dx != dy) {
        axisByVehicle.put(d.name, dx > dy ? Roadmap.DIR_POS_X : Roadmap.DIR_POS_Y);
      }
    }
    return new BodyState(Map.copyOf(axisByVehicle), Set.copyOf(carryingLoad));
  }

  private Vehicle vehicleUnderOpenTcsOrder(String name) {
    Vehicle vehicle = new Vehicle(name);
    Driver driver = vehicles.get(name);
    if (driver == null || driver.goals.isEmpty()) {
      return vehicle;
    }
    return vehicle.withTransportOrder(
        new TransportOrder("order-" + name, List.of()).getReference());
  }

  void addVehicle(String name, String startPoint) {
    Point start = pointByName.get(startPoint);
    if (vehicles.size() >= PARK_POINTS.size()) {
      throw new IllegalStateException(
          "v7-vda5050.xml offers only " + PARK_POINTS.size() + " well-formed park positions;"
              + " cannot place vehicle " + (vehicles.size() + 1)
              + " without making a parked vehicle block a corridor");
    }
    String park = PARK_POINTS.get(vehicles.size());
    if (pointByName.get(park) == null) {
      throw new IllegalStateException("no park point " + park + " on this map");
    }
    if (start == null) {
      throw new IllegalArgumentException("unknown start point '" + startPoint + "'");
    }
    Driver d = new Driver(name, startPoint, park);
    vehicles.put(name, d);
    try {
      scheduler.allocateNow(d, Set.of(start));
    } catch (org.opentcs.components.kernel.ResourceAllocationException e) {
      throw new IllegalStateException("could not place " + name + " at " + startPoint, e);
    }
  }

  void addOrder(String vehicle, String destination) {
    Driver d = vehicles.get(vehicle);
    if (d == null) {
      throw new IllegalArgumentException("unknown vehicle '" + vehicle + "'");
    }
    if (pointByName.get(destination) == null) {
      throw new IllegalArgumentException("unknown destination '" + destination + "'");
    }
    d.goals.add(destination);
    d.goalRanks.add(nextClaimRank++);
    d.waitAfterGoal.add(0);
    d.orders++;
  }

  void addWait(String vehicle, int ticks) {
    Driver d = vehicles.get(vehicle);
    if (d == null) {
      throw new IllegalArgumentException("unknown vehicle '" + vehicle + "'");
    }
    if (d.waitAfterGoal.isEmpty()) {
      throw new IllegalStateException("addWait needs a preceding addOrder for " + vehicle);
    }
    d.waitAfterGoal.addLast(d.waitAfterGoal.removeLast() + ticks);
  }

  MapfCoordinator coordinator() {
    return coordinator;
  }

  String positionOf(String vehicle) {
    return vehicles.get(vehicle).current();
  }

  void enforceContinuity(boolean on) {
    this.enforceContinuity = on;
  }

  boolean firstStepGrantedAfterOptionalReroute(boolean reroute) throws Exception {
    String start = null, oldNext = null, newNext = null;
    for (String pt : new TreeMap<>(pointByName).keySet()) {
      List<String> nb = neighbours(pt);
      if (nb.size() >= 2) {
        start = pt;
        oldNext = nb.get(0);
        newNext = nb.get(1);
        break;
      }
    }
    if (start == null) {
      throw new IllegalStateException("model has no branching point with >=2 neighbours");
    }
    final String oldNextPt = oldNext;
    final boolean[] grantedOld = { false };
    Scheduler.Client client = new Scheduler.Client() {
      @Override
      public String getId() {
        return "RACER";
      }

      @Override
      public TCSObjectReference<Vehicle> getRelatedVehicle() {
        return new Vehicle("RACER").getReference();
      }

      @Override
      public boolean onAllocation(Set<TCSResource<?>> resources) {
        if (resources.contains(pointByName.get(oldNextPt))) {
          grantedOld[0] = true;
        }
        return true;
      }
    };
    scheduler.allocateNow(client, Set.of(pointByName.get(start)));
    scheduler.claim(client, List.of(stepResources(start, oldNext)));
    scheduler.allocate(client, stepResources(start, oldNext));
    if (reroute) {
      scheduler.claim(client, List.of(stepResources(start, newNext)));
    }
    exec.drain();
    log("[FAITHFUL microscope] start=" + start + " oldNext=" + oldNext + " newNext="
        + newNext + " reroute=" + reroute + " -> oldNext granted=" + grantedOld[0]);
    return grantedOld[0];
  }

  private List<String> neighbours(String from) {
    List<String> res = new ArrayList<>();
    for (String q : pointByName.keySet()) {
      if (q.equals(from)) {
        continue;
      }
      if (pathByDir.get(from + ">" + q) != null || pathByDir.get(q + ">" + from) != null) {
        res.add(q);
      }
    }
    res.sort(null);
    return res;
  }

  String journal() {
    return journal.toString();
  }

  int rejectedReroutes() {
    return rejectedReroutes;
  }

  boolean run(int maxTicks) {
    int stall = 0;
    long lastProgressSig = -1;
    long t0 = System.currentTimeMillis();
    resolveAndReclaim(-1);
    exec.drain();
    for (int t = 0; t < maxTicks; t++) {
      tickClock = t;
      if (pendingOrders() == 0) {
        log("[FAITHFUL] all orders completed at tick " + t);
        flush("completed t=" + t);
        return true;
      }
      parkIdleVehicles();
      trace("t=" + t + " ENTER");
      driveAll(t);
      trace("t=" + t + " after driveAll(submit)");
      if (churn || t % HEARTBEAT_TICKS == 0) {
        churn = false;
        resolveAndReclaim(t);
        trace("t=" + t + " after reroute");
      }
      exec.drain();
      trace("t=" + t + " after drain(allocates)");
      freeTrailing();
      exec.drain();
      trace("t=" + t + " after freeTrailing");

      long sig = progressSignature();
      if (sig != lastProgressSig) {
        stall = 0;
        lastProgressSig = sig;
      } else {
        stall++;
      }
      if (t % LOG_EVERY == 0) {
        log("[FAITHFUL t=" + t + " +" + (System.currentTimeMillis() - t0) + "ms] progress="
            + sig + " stall=" + stall + " pending=" + pendingOrders() + " reclaims=" + reclaims
            + " rejectedReroutes=" + rejectedReroutes);
        flush("progress t=" + t);
      }
      if (stall > STALL_LIMIT) {
        log("[FAITHFUL] DEADLOCK at tick " + t + " -- no progress for " + STALL_LIMIT + " ticks");
        dumpState();
        flush("DEADLOCK t=" + t);
        return false;
      }
    }
    log("[FAITHFUL] tick budget exhausted, " + pendingOrders() + " orders pending");
    dumpState();
    flush("budget exhausted");
    return pendingOrders() == 0;
  }

  private void resolveAndReclaim(int t) {
    Map<String, String> positions = new HashMap<>();
    Map<String, String> goals = new HashMap<>();
    Set<String> immobile = new HashSet<>();
    Map<String, Long> claimRank = new HashMap<>();
    for (Driver d : vehicles.values()) {
      positions.put(d.name, d.current());
      if (!d.goals.isEmpty() && !d.waiting(t)) {
        goals.put(d.name, d.goals.peek());
        if (d.goalRanks.peek() != null) {
          claimRank.put(d.name, d.goalRanks.peek());
        }
      } else {
        immobile.add(d.name);
      }
    }
    lastBodyState = bodyState();
    coordinator.resolve(positions, goals, immobile,
        ClaimPolicy.byRank(claimRank), lastBodyState);
    for (Driver d : vehicles.values()) {
      coordinator.plannedGoalOf(d.name)
          .ifPresent(planned -> firstPlannedGoal.putIfAbsent(d.name, planned));
    }
    recordPlansOfWaitingVehicles(t);

    for (Driver d : vehicles.values()) {
      if (d.goals.isEmpty() || d.waiting(t)) {
        continue;
      }
      String from = d.current();
      String goal = d.goals.peek();
      List<String> route = routeServedByProduction(d, from, goal);
      List<String> committedRemaining = d.route.subList(d.idx, d.route.size());
      if (route != null && !route.equals(committedRemaining)) {
        if (enforceContinuity
            && !RerouteContinuityService.canApply(d.sentNotYetExecuted(), route)) {
          rejectedReroutes++;
          continue;
        }
        reclaims++;
        log("[RECLAIM t=" + t + "] " + d.name + " @" + from + " idx=" + d.idx
            + " OLD=" + committedRemaining + " NEW=" + route);
        d.assignRoute(route);
      }
    }
  }

  private List<String> routeServedByProduction(Driver d, String from, String goal) {
    Optional<List<String>> served = coordinator.pointsFor(d.name, from, goal, ClaimPolicy.DEFAULT)
        .filter(points -> points.size() >= 2)
        .filter(points -> points.get(points.size() - 1).equals(goal));
    return served.isPresent() ? new ArrayList<>(served.get()) : null;
  }

  private void driveAll(int t) {
    for (Driver d : vehicles.values()) {
      d.driveTick(t);
    }
  }

  private void freeTrailing() {
    Map<String, Set<TCSResource<?>>> allocs = scheduler.getAllocations();
    for (Driver d : vehicles.values()) {
      Point here = pointByName.get(d.current());
      Set<TCSResource<?>> held = allocs.getOrDefault(d.name, Set.of());
      Set<TCSResource<?>> behind = new HashSet<>(held);
      behind.remove(here);
      if (!behind.isEmpty()) {
        scheduler.free(d, behind);
      }
    }
  }

  private Set<TCSResource<?>> stepResources(String from, String to) {
    Set<TCSResource<?>> set = new HashSet<>();
    set.add(pointByName.get(to));
    Path p = pathByDir.get(from + ">" + to);
    if (p == null) {
      p = pathByDir.get(to + ">" + from);
    }
    if (p != null) {
      set.add(p);
    }
    return set;
  }

  private void recordPlansOfWaitingVehicles(int t) {
    for (Driver d : vehicles.values()) {
      if (!d.waiting(t)) {
        continue;
      }
      int planned = coordinator.cachedRoute(d.name, d.current())
          .map(List::size)
          .orElse(Integer.MAX_VALUE);
      if (planned > 1) {
        log("[WAITLOCK t=" + t + "] " + d.name + " @" + d.current() + " planned="
            + coordinator.cachedRoute(d.name, d.current()).orElse(List.of()));
      }
      maxPlannedWhileWaiting.merge(d.name, planned, Math::max);
    }
  }

  int maxPlannedRouteWhileWaiting(String vehicle) {
    return maxPlannedWhileWaiting.getOrDefault(vehicle, 0);
  }

  private void logStall(int t, Driver d, String want) {
    String occupant = "-";
    for (Driver other : vehicles.values()) {
      if (other != d && other.current().equals(want)) {
        occupant = other.name;
        break;
      }
    }
    Adg adg = coordinator.adg();
    int step = adg.routeOf(d.name).indexOf(want);
    List<Step> blockers =
        step < 0 ? List.of() : adg.predecessorsOf(new Step(d.name, step));
    log("[STALL t=" + t + "] " + d.name + " @" + d.current() + " wants=" + want
        + " occupiedBy=" + occupant
        + " blockersOfWant=" + blockers
        + " planRoute=" + adg.routeOf(d.name));
  }

  private int pendingOrders() {
    return vehicles.values().stream().mapToInt(d -> d.orders).sum();
  }

  private void parkIdleVehicles() {
    for (Driver d : vehicles.values()) {
      if (d.orders == 0 && d.goals.isEmpty() && !d.current().equals(d.parkPoint)) {
        d.goals.add(d.parkPoint);
        d.goalRanks.add(Long.MAX_VALUE);
      }
    }
  }

  private long progressSignature() {
    long sig = 0;
    for (Driver d : vehicles.values()) {
      sig += d.moves + 10_000L * d.completed + d.waitedTicks;
    }
    return sig;
  }

  private void dumpState() {
    Map<String, Set<TCSResource<?>>> allocs = scheduler.getAllocations();
    for (Driver d : vehicles.values()) {
      log("[FAITHFUL]   " + d.name + " @" + d.current() + " idx=" + d.idx
          + "/" + (d.route.size() - 1) + " goals=" + d.goals
          + " waiting=" + (d.pending == null ? "-" : d.pending)
          + " alloc=" + allocs.getOrDefault(d.name, Set.of()));
    }
  }

  private void log(String line) {
    journal.append(line).append('\n');
    System.out.println(line);
  }

  private void trace(String line) {
    journal.append("  . ").append(line).append('\n');
  }

  private void flush(String outcome) {
    try {
      File out = new File("build/faithful-sim-journal.log").getAbsoluteFile();
      out.getParentFile().mkdirs();
      java.nio.file.Files.writeString(out.toPath(),
          "=== run " + java.time.LocalTime.now() + " (" + outcome + ") ===\n" + journal,
          java.nio.file.StandardOpenOption.CREATE,
          java.nio.file.StandardOpenOption.TRUNCATE_EXISTING);
    } catch (Exception e) {
      System.out.println("[FAITHFUL] could not write journal: " + e);
    }
  }

  private final class Driver
      implements
      Scheduler.Client {

    private final String name;
    private final TCSObjectReference<Vehicle> ref;
    List<String> route;
    int idx;
    final Deque<String> goals = new ArrayDeque<>();
    final Deque<Long> goalRanks = new ArrayDeque<>();
    final Deque<Integer> waitAfterGoal = new ArrayDeque<>();
    int waitUntilTick = Integer.MIN_VALUE;
    long waitedTicks;
    final String parkPoint;
    int orders;
    int completed;
    long moves;
    boolean movedThisLeg;
    Set<TCSResource<?>> pending;

    Driver(String name, String startPoint, String parkPoint) {
      this.parkPoint = parkPoint;
      this.name = name;
      this.ref = new Vehicle(name).getReference();
      this.route = new ArrayList<>(List.of(startPoint));
      this.idx = 0;
    }

    String current() {
      return route.get(idx);
    }

    List<String> sentNotYetExecuted() {
      if (pending == null || idx + 1 >= route.size()) {
        return List.of(current());
      }
      return List.of(current(), route.get(idx + 1));
    }

    void assignRoute(List<String> newRoute) {
      pending = null;
      this.route = new ArrayList<>(newRoute);
      this.idx = 0;
      List<Set<TCSResource<?>>> claim = new ArrayList<>();
      for (int i = 0; i + 1 < route.size(); i++) {
        claim.add(stepResources(route.get(i), route.get(i + 1)));
      }
      scheduler.claim(this, claim);
    }

    boolean waiting(int t) {
      return t < waitUntilTick;
    }

    void driveTick(int t) {
      if (waiting(t)) {
        waitedTicks++;
        return;
      }
      if (!goals.isEmpty() && current().equals(goals.peek())) {
        goals.poll();
        goalRanks.poll();
        int waitTicks = waitAfterGoal.isEmpty() ? 0 : waitAfterGoal.poll();
        if (waitTicks > 0) {
          waitUntilTick = t + waitTicks;
        }
        if (orders > 0) {
          orders--;
        }
        completed++;
        movedThisLeg = false;
        churn = true;
        log("[FAITHFUL t=" + t + "] " + name + " COMPLETED at " + current()
            + " (" + goals.size() + " to go)");
        return;
      }
      if (idx + 1 >= route.size()) {
        return;
      }
      if (pending != null) {
        logStall(t, this, route.get(idx + 1));
        return;
      }
      Set<TCSResource<?>> next = stepResources(current(), route.get(idx + 1));
      pending = next;

      scheduler.allocate(this, next);
    }

    @Override
    public boolean onAllocation(Set<TCSResource<?>> resources) {

      pending = null;
      idx++;
      moves++;
      movedThisLeg = true;
      return true;
    }

    @Override
    public String getId() {
      return name;
    }

    @Override
    public TCSObjectReference<Vehicle> getRelatedVehicle() {
      return ref;
    }
  }

  private static final class SameThreadScheduledExecutor
      extends AbstractExecutorService
      implements ScheduledExecutorService {

    private final java.util.ArrayDeque<Runnable> queue = new java.util.ArrayDeque<>();

    @Override
    public void execute(Runnable command) {
      queue.add(command);
    }

    void drain() {
      Runnable r;
      while ((r = queue.poll()) != null) {
        r.run();
      }
    }

    @Override
    public void shutdown() {
    }

    @Override
    public List<Runnable> shutdownNow() {
      return List.of();
    }

    @Override
    public boolean isShutdown() {
      return false;
    }

    @Override
    public boolean isTerminated() {
      return false;
    }

    @Override
    public boolean awaitTermination(long timeout, TimeUnit unit) {
      return true;
    }

    @Override
    public ScheduledFuture<?> schedule(Runnable command, long delay, TimeUnit unit) {
      return new NeverFiringSchedule<>();
    }

    @Override
    public <V> ScheduledFuture<V> schedule(Callable<V> callable, long delay, TimeUnit unit) {
      return new NeverFiringSchedule<>();
    }

    @Override
    public ScheduledFuture<?> scheduleAtFixedRate(
        Runnable command, long initialDelay, long period, TimeUnit unit) {
      return new NeverFiringSchedule<>();
    }

    @Override
    public ScheduledFuture<?> scheduleWithFixedDelay(
        Runnable command, long initialDelay, long delay, TimeUnit unit) {
      return new NeverFiringSchedule<>();
    }
  }

  private static final class NeverFiringSchedule<V>
      implements
        ScheduledFuture<V> {

    private volatile boolean cancelled;

    @Override
    public long getDelay(TimeUnit unit) {
      return Long.MAX_VALUE;
    }

    @Override
    public int compareTo(Delayed other) {
      return Long.compare(
          getDelay(TimeUnit.MILLISECONDS), other.getDelay(TimeUnit.MILLISECONDS));
    }

    @Override
    public boolean cancel(boolean mayInterruptIfRunning) {
      cancelled = true;
      return true;
    }

    @Override
    public boolean isCancelled() {
      return cancelled;
    }

    @Override
    public boolean isDone() {
      return cancelled;
    }

    @Override
    public V get() {
      throw new CancellationException();
    }

    @Override
    public V get(long timeout, TimeUnit unit) {
      throw new CancellationException();
    }
  }
}
