package org.opentcs.fms.kernel.mapf;

import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.ArrayList;
import java.util.function.LongSupplier;

import org.opentcs.fms.kernel.mapf.adg.Adg;
import org.opentcs.fms.kernel.mapf.adg.Step;

final class VetoLogger {

  private static final org.slf4j.Logger logger =
      org.slf4j.LoggerFactory.getLogger(VetoLogger.class);

  private static final int MISMATCHES_SHOWN = 4;
  private static final String UNPLANNED_TAIL = "the unplanned tail";
  private static final String UNPLANNED_CLAIM = "a solve covering its new claim";

  private record Episode(long startNanos, String blocker) {
  }

  private volatile LongSupplier nanoClock = System::nanoTime;

  private long generation = -1L;
  private int fellBackToClaims;
  private int askedWithNoGraph;
  private int askedForSomethingElse;
  private int mismatchesShown;
  private int askedWithNoBlocker;
  private int refusals;
  private int horizonRefusals;
  private int offPlanRefusals;
  private final Map<String, Episode> episodes = new HashMap<>();

  void setClock(LongSupplier nanoClock) {
    this.nanoClock = nanoClock;
  }

  void noGraph() {
    askedWithNoGraph++;
  }

  void fellBackToClaims() {
    fellBackToClaims++;
  }

  boolean startedNewGeneration(Adg adg) {
    if (adg.generation() == generation) {
      return false;
    }
    if (generation >= 0) {
      logger.info(
          "[MAPF prec-gen] generation {} -> {} | mayAllocate: noGraph={} notMyNextPoint={}"
              + " noBlocker={} HELD={} BEYOND-HORIZON={} OFF-PLAN={}"
              + " | {} step lookups fell back to claims",
          generation, adg.generation(), askedWithNoGraph, askedForSomethingElse,
          askedWithNoBlocker, refusals, horizonRefusals, offPlanRefusals, fellBackToClaims);
    }
    generation = adg.generation();
    fellBackToClaims = 0;
    askedWithNoGraph = 0;
    askedForSomethingElse = 0;
    mismatchesShown = 0;
    askedWithNoBlocker = 0;
    refusals = 0;
    horizonRefusals = 0;
    offPlanRefusals = 0;
    dropEpisodesOutsideThePlan(adg);
    return true;
  }

  void askedForAnotherPoint(
      Adg adg, Step wanted, String nextPoint, String standingAt, List<String> requested,
      int lastKnownStep, List<String> claim) {
    askedForSomethingElse++;
    if (mismatchesShown++ < MISMATCHES_SHOWN) {
      List<String> route = adg.routeOf(wanted.vehicle());
      logger.info(
          "[MAPF prec-silent] {} standing at {} asked for {} but its next step {} is {}:"
              + " no edge consulted | lastKnown={} | route({})={} | claim({})={}"
              + " | route-claim={}",
          wanted.vehicle(), standingAt, requested, wanted, nextPoint,
          lastKnownStep, route.size(), route, claim.size(), claim,
          route.size() - claim.size());
    }
    endEpisode(wanted.vehicle());
  }

  void noBlocker(String vehicle) {
    askedWithNoBlocker++;
    endEpisode(vehicle);
  }

  boolean refused(Step wanted, Step blocker) {
    refusals++;
    Episode opening = new Episode(nanoClock.getAsLong(), blocker.vehicle());
    return episodes.putIfAbsent(wanted.vehicle(), opening) == null;
  }

  boolean refusedBeyondHorizon(String vehicle) {
    horizonRefusals++;
    Episode opening = new Episode(nanoClock.getAsLong(), UNPLANNED_TAIL);
    return episodes.putIfAbsent(vehicle, opening) == null;
  }

  void describeHorizonHold(Adg adg, Step wanted, String point) {
    logger.info(
        "[MAPF horizon] {} may not enter {} (step {} of {}): the fleet plan is only proven up to"
            + " step {}. Everything past that was appended by the goal padding, not by the solver,"
            + " and the next solve replaces it | adgGen={} ageMs={}",
        wanted.vehicle(), point, wanted.step(), adg.routeOf(wanted.vehicle()).size(),
        adg.coordinatedStepOf(wanted.vehicle()), adg.generation(), ageMs(adg));
  }

  boolean refusedOffPlan(String vehicle) {
    offPlanRefusals++;
    Episode opening = new Episode(nanoClock.getAsLong(), UNPLANNED_CLAIM);
    return episodes.putIfAbsent(vehicle, opening) == null;
  }

  void describeOffPlanHold(
      Adg adg, String vehicle, String standingAt, List<String> requested, List<String> claim) {
    List<String> route = adg.routeOf(vehicle);
    logger.info(
        "[MAPF off-plan] {} standing at {} may not enter {}: what openTCS still claims for it"
            + " is not the rest of its plan route, so the request belongs to a drive order the"
            + " plan never covered; the next solve decides | route({})={} | claim({})={}"
            + " | adgGen={} ageMs={}",
        vehicle, standingAt, requested, route.size(), route, claim.size(), claim,
        adg.generation(), ageMs(adg));
  }

  void describeHold(
      Adg adg, Step wanted, Step blocker, String waiterAt, String blockerAt,
      int blockerNextStep, int blockerLastKnown) {
    logger.info(
        "[MAPF veto] {} standing at {} may not enter {} ({}) yet:"
            + " {} has not left {} ({}) | blocker@{} step={}/{} lastKnown={}"
            + " | adgGen={} ageMs={}",
        wanted.vehicle(), waiterAt, adg.pointOf(wanted), wanted,
        blocker.vehicle(), adg.pointOf(blocker), blocker,
        blockerAt, blockerNextStep, adg.routeOf(blocker.vehicle()).size(), blockerLastKnown,
        adg.generation(), ageMs(adg));
  }

  void released(String vehicle) {
    endEpisode(vehicle);
  }

  List<String> openEpisodes() {
    List<String> open = new ArrayList<>();
    episodes.entrySet().stream()
        .sorted(Map.Entry.comparingByKey())
        .forEach(entry -> open.add(entry.getKey() + " waiting for "
            + entry.getValue().blocker() + " for "
            + millisSince(entry.getValue().startNanos()) + "ms"));
    return List.copyOf(open);
  }

  private void endEpisode(String vehicle) {
    Episode episode = episodes.remove(vehicle);
    if (episode != null) {
      logger.info("[MAPF veto-end] {} released after {}ms (waited for {})",
          vehicle, millisSince(episode.startNanos()), episode.blocker());
    }
  }

  private void dropEpisodesOutsideThePlan(Adg adg) {
    Iterator<Map.Entry<String, Episode>> stale = episodes.entrySet().iterator();
    while (stale.hasNext()) {
      Map.Entry<String, Episode> entry = stale.next();
      if (adg.routes().containsKey(entry.getKey())) {
        continue;
      }
      logger.info("[MAPF veto-end] {} dropped after {}ms (waited for {}, no longer in the plan)",
          entry.getKey(), millisSince(entry.getValue().startNanos()), entry.getValue().blocker());
      stale.remove();
    }
  }

  private long millisSince(long startNanos) {
    return (nanoClock.getAsLong() - startNanos) / 1_000_000L;
  }

  private static String ageMs(Adg adg) {
    return adg.builtAtMs() == 0L
        ? "never-built"
        : String.valueOf(System.currentTimeMillis() - adg.builtAtMs());
  }
}
