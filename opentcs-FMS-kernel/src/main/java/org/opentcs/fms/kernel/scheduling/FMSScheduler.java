package org.opentcs.fms.kernel.scheduling;

import jakarta.inject.Inject;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;
import org.opentcs.customizations.ApplicationEventBus;
import org.opentcs.customizations.kernel.GlobalSyncObject;
import org.opentcs.customizations.kernel.KernelExecutor;
import org.opentcs.data.model.TCSResource;
import org.opentcs.fms.kernel.FleetSnapshotService;
import org.opentcs.fms.kernel.mapf.MapfCaptureStore;
import org.opentcs.fms.kernel.mapf.MapfPrecedenceModule;
import org.opentcs.fms.kernel.mapf.adg.Adg;
import org.opentcs.fms.kernel.mapf.adg.AdgService;
import org.opentcs.fms.kernel.mapf.adg.Step;
import org.opentcs.strategies.basic.scheduling.AllocationAdvisor;
import org.opentcs.strategies.basic.scheduling.DefaultScheduler;
import org.opentcs.strategies.basic.scheduling.ReservationPool;
import org.opentcs.util.event.EventBus;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class FMSScheduler
    extends
    DefaultScheduler {

  private static final Logger LOG = LoggerFactory.getLogger(FMSScheduler.class);

  private static final long STALL_SCAN_INTERVAL_MS = 5000;
  private static final long STALL_REPORT_THRESHOLD_MS = 15000;
  private static final long STALL_DUMP_THRESHOLD_MS = Math.max(
      1L, Long.getLong("fms.mapf.stallDumpSeconds", 25L)) * 1000L;
  private static final int CLAIM_CHURN_REPORT_EVERY_SCANS = 6;

  private final ReservationPool reservationPool;
  private final Object globalSyncObject;
  private final ScheduledExecutorService kernelExecutor;
  private final FleetSnapshotService fleetSnapshotService;
  private final AdgService adgService;
  private final MapfPrecedenceModule precedenceModule;

  private final Map<String, Client> clientsById = new HashMap<>();
  private final Map<String, String> lastStateSignatureById = new HashMap<>();
  private final Map<String, Long> stateUnchangedSinceById = new HashMap<>();
  private final Map<String, Long> lastAllocateRequestAtById = new HashMap<>();
  private final Map<String, Long> lastPurgeAtById = new HashMap<>();
  private final Map<String, Map<String, Integer>> claimTransitionCountsById = new HashMap<>();
  private final Map<String, String> lastReportedPositionByVehicle = new HashMap<>();
  private final Map<String, String> lastReportedGoalByVehicle = new HashMap<>();
  private final Map<String, Long> positionUnchangedSinceByVehicle = new HashMap<>();
  private final Set<String> dumpedStallEpisodes = new HashSet<>();

  private ScheduledFuture<?> stallScan;
  private int scansSinceClaimChurnReport;

  private record StallDumpState(
      String stalledVehicle, long positionUnchangedSeconds, String position, String goal,
      Map<String, String> positions, Map<String, String> goals,
      Map<String, List<String>> installedRoutes,
      Map<String, List<List<String>>> claimedResources,
      Map<String, List<String>> allocatedResources,
      List<String> adgEdges, List<String> openVetoes) {
  }

  @Inject
  public FMSScheduler(
      AllocationAdvisor allocationAdvisor,
      ReservationPool reservationPool,
      @KernelExecutor ScheduledExecutorService kernelExecutor,
      @ApplicationEventBus EventBus eventBus,
      @GlobalSyncObject Object globalSyncObject,
      FleetSnapshotService fleetSnapshotService, AdgService adgService,
      MapfPrecedenceModule precedenceModule) {
    super(allocationAdvisor, reservationPool, kernelExecutor, eventBus, globalSyncObject);
    this.reservationPool = reservationPool;
    this.globalSyncObject = globalSyncObject;
    this.kernelExecutor = kernelExecutor;
    this.fleetSnapshotService = fleetSnapshotService;
    this.adgService = adgService;
    this.precedenceModule = precedenceModule;
  }

  public FMSScheduler(
      AllocationAdvisor allocationAdvisor,
      ReservationPool reservationPool,
      ScheduledExecutorService kernelExecutor,
      EventBus eventBus,
      Object globalSyncObject) {
    this(allocationAdvisor, reservationPool, kernelExecutor, eventBus, globalSyncObject,
        null, null, null);
  }

  @Override
  public void initialize() {
    boolean alreadyInitialized = isInitialized();
    super.initialize();

    if (!alreadyInitialized && isInitialized()) {
      stallScan = kernelExecutor.scheduleAtFixedRate(
          this::reportStalledClients,
          STALL_SCAN_INTERVAL_MS,
          STALL_SCAN_INTERVAL_MS,
          TimeUnit.MILLISECONDS);
    }
  }

  @Override
  public void terminate() {
    boolean wasInitialized = isInitialized();
    super.terminate();

    if (wasInitialized && !isInitialized() && stallScan != null) {
      stallScan.cancel(false);
      stallScan = null;
      forgetTracking();
    }
  }

  @Override
  public void claim(Client client, List<Set<TCSResource<?>>> resources) {
    synchronized (globalSyncObject) {
      track(client);
      lastPurgeAtById.put(client.getId(), System.currentTimeMillis());
      countClaimTransition(client, reservationPool.getClaim(client), resources);
      clearPendingAllocations(client);
      super.claim(client, resources);
    }
  }

  private void countClaimTransition(
      Client client,
      List<Set<TCSResource<?>>> previousClaim,
      List<Set<TCSResource<?>>> newClaim) {
    claimTransitionCountsById
        .computeIfAbsent(client.getId(), id -> new HashMap<>())
        .merge(claimTransition(previousClaim, newClaim), 1, Integer::sum);
  }

  private static String claimTransition(
      List<Set<TCSResource<?>>> previousClaim,
      List<Set<TCSResource<?>>> newClaim) {
    if (newClaim.isEmpty()) {
      return previousClaim.isEmpty() ? "identical" : "changed";
    }
    for (int offset = 0; offset <= previousClaim.size() - newClaim.size(); offset++) {
      if (previousClaim.subList(offset, offset + newClaim.size()).equals(newClaim)) {
        return offset == 0 ? "identical" : "advanced";
      }
    }
    return "changed";
  }

  @Override
  public void allocate(Client client, Set<TCSResource<?>> resources) {
    synchronized (globalSyncObject) {
      track(client);
      lastAllocateRequestAtById.put(client.getId(), System.currentTimeMillis());

      if (!reservationPool.isNextInClaim(client, resources)) {
        resyncClaimTo(client, resources);
      }
      super.allocate(client, resources);
    }
  }

  private void resyncClaimTo(Client client, Set<TCSResource<?>> resources) {
    List<Set<TCSResource<?>>> staleClaim = reservationPool.getClaim(client);
    int requestedIndex = staleClaim.indexOf(resources);

    if (requestedIndex < 0) {
      LOG.warn(
          "{}: requested allocation {} is not on the claimed route at all; claiming it on its own."
              + " Lookahead is lost until the next drive order update.",
          client.getId(),
          resources);
    } else {
      LOG.warn(
          "{}: claim lags the requested allocation {} by {} entries; dropping the stale prefix.",
          client.getId(),
          resources,
          requestedIndex);
    }

    List<Set<TCSResource<?>>> resyncedClaim = requestedIndex < 0
        ? List.of(resources)
        : List.copyOf(staleClaim.subList(requestedIndex, staleClaim.size()));

    clearPendingAllocations(client);
    claim(client, resyncedClaim);
  }

  private void track(Client client) {
    clientsById.put(client.getId(), client);
  }

  private void forgetTracking() {
    clientsById.clear();
    lastStateSignatureById.clear();
    stateUnchangedSinceById.clear();
    lastAllocateRequestAtById.clear();
    lastPurgeAtById.clear();
    claimTransitionCountsById.clear();
    lastReportedPositionByVehicle.clear();
    lastReportedGoalByVehicle.clear();
    positionUnchangedSinceByVehicle.clear();
    dumpedStallEpisodes.clear();
    scansSinceClaimChurnReport = 0;
  }

  @SuppressWarnings("IllegalCatch")
  private void reportStalledClients() {
    try {
      synchronized (globalSyncObject) {
        long now = System.currentTimeMillis();
        Map<String, Set<TCSResource<?>>> allocationsByClientId = reservationPool.getAllocations();
        Map<String, String> holderByResourceName = holderByResourceName(allocationsByClientId);

        if (fleetSnapshotService != null) {
          try {
            reportPositionStalls(now, fleetSnapshotService.read(), allocationsByClientId);
          } catch (RuntimeException e) {
            LOG.warn("[MAPF stall-dump] scan failed", e);
          }
        }

        for (Client client : List.copyOf(clientsById.values())) {
          reportIfStalled(client, now, allocationsByClientId, holderByResourceName);
        }

        if (++scansSinceClaimChurnReport >= CLAIM_CHURN_REPORT_EVERY_SCANS) {
          scansSinceClaimChurnReport = 0;
          reportClaimChurn();
        }
      }
    } catch (RuntimeException e) {
      LOG.warn("[alloc stall] scan failed", e);
    }
  }

  private void reportPositionStalls(
      long now, FleetSnapshotService.Fleet fleet,
      Map<String, Set<TCSResource<?>>> allocationsByClientId) {
    Set<String> presentVehicles = fleet.positions().keySet();
    lastReportedPositionByVehicle.keySet().retainAll(presentVehicles);
    lastReportedGoalByVehicle.keySet().retainAll(presentVehicles);
    positionUnchangedSinceByVehicle.keySet().retainAll(presentVehicles);
    dumpedStallEpisodes.retainAll(presentVehicles);
    for (Map.Entry<String, String> entry : fleet.positions().entrySet()) {
      String vehicle = entry.getKey();
      String position = entry.getValue();
      String goal = fleet.goals().get(vehicle);
      String previousPosition = lastReportedPositionByVehicle.put(vehicle, position);
      String previousGoal = lastReportedGoalByVehicle.put(vehicle, goal);
      if (previousPosition == null || !previousPosition.equals(position)
          || (!hasRealGoal(position, previousGoal) && hasRealGoal(position, goal))) {
        positionUnchangedSinceByVehicle.put(vehicle, now);
        dumpedStallEpisodes.remove(vehicle);
      }
      if (!hasRealGoal(position, goal)) {
        dumpedStallEpisodes.remove(vehicle);
        continue;
      }
      long unchangedSince = positionUnchangedSinceByVehicle.getOrDefault(vehicle, now);
      if (now - unchangedSince <= STALL_DUMP_THRESHOLD_MS || !dumpedStallEpisodes.add(vehicle)) {
        continue;
      }
      StallDumpState state = stallDumpState(
          vehicle, now, unchangedSince, position, goal, fleet, allocationsByClientId);
      var path = MapfCaptureStore.dumpStall("stall-" + vehicle, state);
      if (path != null) {
        LOG.warn("[MAPF stall-dump] {} position={} goal={} unchanged={}s dump={}",
            vehicle, position, goal, (now - unchangedSince) / 1000, path);
      }
    }
  }

  private StallDumpState stallDumpState(
      String stalledVehicle, long now, long unchangedSince, String position, String goal,
      FleetSnapshotService.Fleet fleet,
      Map<String, Set<TCSResource<?>>> allocationsByClientId) {
    Map<String, List<List<String>>> claims = new TreeMap<>();
    for (Map.Entry<String, Client> entry : clientsById.entrySet()) {
      List<List<String>> steps = new ArrayList<>();
      for (Set<TCSResource<?>> step : reservationPool.getClaim(entry.getValue())) {
        steps.add(step.stream().map(TCSResource::getName).sorted().toList());
      }
      claims.put(entry.getKey(), List.copyOf(steps));
    }
    Map<String, List<String>> allocations = new TreeMap<>();
    allocationsByClientId.forEach((vehicle, resources) -> allocations.put(
        vehicle, resources.stream().map(TCSResource::getName).sorted().toList()));
    Adg adg = adgService == null ? Adg.EMPTY : adgService.adg();
    List<String> edges = new ArrayList<>();
    adg.dependencies().forEach((waiter, blockers) -> {
      if (!waiter.vehicle().equals(stalledVehicle)
          && blockers.stream().noneMatch(blocker -> blocker.vehicle().equals(stalledVehicle))) {
        return;
      }
      for (Step blocker : blockers) {
        edges.add(waiter + "@" + adg.pointOf(waiter) + "<-" + blocker + "@" + adg.pointOf(blocker));
      }
    });
    edges.sort(null);
    Map<String, List<String>> routes = fleetSnapshotService.installedRoutes();
    List<String> openVetoes = precedenceModule == null
        ? List.of()
        : precedenceModule.openVetoes();
    return new StallDumpState(stalledVehicle, (now - unchangedSince) / 1000,
        position, goal, Map.copyOf(fleet.positions()), Map.copyOf(fleet.goals()), routes,
        Map.copyOf(claims), Map.copyOf(allocations), List.copyOf(edges), openVetoes);
  }

  private static boolean hasRealGoal(String position, String goal) {
    return goal != null && !goal.equals(position);
  }

  private void reportClaimChurn() {
    if (claimTransitionCountsById.isEmpty()) {
      return;
    }

    Map<String, Integer> totals = new HashMap<>();
    claimTransitionCountsById.values().forEach(
        counts -> counts.forEach(
            (transition, count) -> totals.merge(transition, count, Integer::sum)));

    int identical = totals.getOrDefault("identical", 0);
    int advanced = totals.getOrDefault("advanced", 0);
    int changed = totals.getOrDefault("changed", 0);
    int all = identical + advanced + changed;

    LOG.info(
        "[claim churn] last {}s: {} claims | identical={} advanced={} changed={}"
            + " | purged-for-nothing={}% | per-vehicle i/a/c: {}",
        STALL_SCAN_INTERVAL_MS * CLAIM_CHURN_REPORT_EVERY_SCANS / 1000,
        all,
        identical,
        advanced,
        changed,
        all == 0 ? 0 : (identical + advanced) * 100 / all,
        perVehicleClaimChurn());

    claimTransitionCountsById.clear();
  }

  private String perVehicleClaimChurn() {
    return claimTransitionCountsById.entrySet().stream()
        .sorted(Map.Entry.comparingByKey())
        .map(entry -> entry.getKey() + "=" + entry.getValue().getOrDefault("identical", 0)
            + "/" + entry.getValue().getOrDefault("advanced", 0)
            + "/" + entry.getValue().getOrDefault("changed", 0))
        .collect(Collectors.joining(" "));
  }

  private void reportIfStalled(
      Client client,
      long now,
      Map<String, Set<TCSResource<?>>> allocationsByClientId,
      Map<String, String> holderByResourceName) {
    String clientId = client.getId();
    List<Set<TCSResource<?>>> claim = reservationPool.getClaim(client);
    Set<TCSResource<?>> allocated = allocationsByClientId.getOrDefault(clientId, Set.of());

    String signature = namesOf(claim.isEmpty() ? Set.of() : claim.get(0))
        + "|" + namesOf(allocated);
    if (!signature.equals(lastStateSignatureById.put(clientId, signature))) {
      stateUnchangedSinceById.put(clientId, now);
      return;
    }

    long stalledMs = now - stateUnchangedSinceById.getOrDefault(clientId, now);
    if (claim.isEmpty() || stalledMs < STALL_REPORT_THRESHOLD_MS) {
      return;
    }

    LOG.info(
        "[alloc stall] {}: {}s | wants={} heldBy={} | allocated={} claimLen={}"
            + " | lastAllocateRequest={} lastPurge={}",
        clientId,
        stalledMs / 1000,
        namesOf(claim.get(0)),
        holdersOf(claim.get(0), holderByResourceName),
        namesOf(allocated),
        claim.size(),
        ageOf(lastAllocateRequestAtById.get(clientId), now),
        ageOf(lastPurgeAtById.get(clientId), now));
  }

  private Map<String, String> holderByResourceName(
      Map<String, Set<TCSResource<?>>> allocationsByClientId) {
    Map<String, String> result = new HashMap<>();
    allocationsByClientId.forEach(
        (clientId, resources) -> resources.forEach(
            resource -> result.put(resource.getName(), clientId)));
    return result;
  }

  private String holdersOf(
      Set<TCSResource<?>> resources,
      Map<String, String> holderByResourceName) {
    return resources.stream()
        .map(TCSResource::getName)
        .sorted()
        .map(name -> name + "=" + holderByResourceName.getOrDefault(name, "free"))
        .collect(Collectors.joining(","));
  }

  private String namesOf(Set<TCSResource<?>> resources) {
    return resources.stream()
        .map(TCSResource::getName)
        .sorted()
        .collect(Collectors.joining(","));
  }

  private String ageOf(Long timestamp, long now) {
    return timestamp == null ? "never" : ((now - timestamp) / 1000) + "s";
  }
}
