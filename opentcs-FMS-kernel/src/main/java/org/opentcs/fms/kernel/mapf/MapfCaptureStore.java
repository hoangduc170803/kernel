package org.opentcs.fms.kernel.mapf;

import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.time.ZoneOffset;
import java.time.format.DateTimeFormatter;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.opentcs.data.order.Route;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class MapfCaptureStore {

  public record CaptureDump(String trigger, String capturedAt, List<MapfSolveCapture> captures) {
  }

  public record StallDump(
      String trigger, String capturedAt, List<MapfSolveCapture> captures, Object stallState) {
  }

  public record RouteCommitment(String vehicle, List<String> solverCommittedPrefix) {
    public RouteCommitment {
      solverCommittedPrefix = List.copyOf(solverCommittedPrefix);
    }
  }

  private static final Logger LOG = LoggerFactory.getLogger(MapfCaptureStore.class);
  private static final int CAPACITY = 20;
  private static final int ROUTE_COMMITMENT_CAPACITY = 256;
  private static final long MIN_DUMP_GAP_MS = 30_000L;
  private static final long SPLICE_REVISIT_DUMP_GAP_MS = 10_000L;
  private static final DateTimeFormatter FILE_STAMP = DateTimeFormatter
      .ofPattern("yyyyMMdd-HHmmss-SSS").withZone(ZoneOffset.UTC);
  private static final ObjectMapper JSON = new ObjectMapper();
  private static final Deque<MapfSolveCapture> RECENT = new ArrayDeque<>(CAPACITY);
  private static final Deque<Route> ROUTE_COMMITMENT_ORDER = new ArrayDeque<>();
  private static final Map<Route, RouteCommitment> ROUTE_COMMITMENTS = new IdentityHashMap<>();
  private static final Map<String, Long> LAST_DUMP_BY_TRIGGER = new HashMap<>();

  private MapfCaptureStore() {
  }

  public static synchronized void record(MapfSolveCapture capture) {
    if (RECENT.size() == CAPACITY) {
      RECENT.removeFirst();
    }
    RECENT.addLast(capture);
  }

  public static synchronized List<MapfSolveCapture> recentCaptures() {
    return List.copyOf(RECENT);
  }

  public static synchronized void registerRouteCommitment(
      Route route, String vehicle, List<String> solverCommittedPrefix) {
    if (!ROUTE_COMMITMENTS.containsKey(route)) {
      ROUTE_COMMITMENT_ORDER.addLast(route);
    }
    ROUTE_COMMITMENTS.put(route,
        new RouteCommitment(vehicle, solverCommittedPrefix));
    while (ROUTE_COMMITMENT_ORDER.size() > ROUTE_COMMITMENT_CAPACITY) {
      ROUTE_COMMITMENTS.remove(ROUTE_COMMITMENT_ORDER.removeFirst());
    }
  }

  public static synchronized Optional<RouteCommitment> takeRouteCommitment(
      Route route, String vehicle) {
    RouteCommitment commitment = ROUTE_COMMITMENTS.get(route);
    if (commitment == null || !commitment.vehicle().equals(vehicle)) {
      return Optional.empty();
    }
    ROUTE_COMMITMENTS.remove(route);
    Iterator<Route> routes = ROUTE_COMMITMENT_ORDER.iterator();
    while (routes.hasNext()) {
      if (routes.next() == route) {
        routes.remove();
        break;
      }
    }
    return Optional.of(commitment);
  }

  public static void dumpIfAllowed(MapfSolveCapture capture, String trigger) {
    long now = System.currentTimeMillis();
    if (!claimDumpSlot(trigger, now, MIN_DUMP_GAP_MS)) {
      return;
    }
    write("mapf-capture", safeName(trigger),
        new CaptureDump(trigger, Instant.now().toString(), List.of(capture)));
  }

  public static void dumpRecentIfAllowed(String trigger) {
    long now = System.currentTimeMillis();
    if (!claimDumpSlot("splice-revisit-global", now, SPLICE_REVISIT_DUMP_GAP_MS)) {
      return;
    }
    write("mapf-capture", safeName(trigger),
        new CaptureDump(trigger, Instant.now().toString(), recentCaptures()));
  }

  public static Path dumpStall(String trigger, Object stallState) {
    return write("mapf-capture", safeName(trigger),
        new StallDump(trigger, Instant.now().toString(), recentCaptures(), stallState), false);
  }

  private static void write(String childDirectory, String trigger, Object value) {
    write(childDirectory, trigger, value, true);
  }

  private static Path write(
      String childDirectory, String trigger, Object value, boolean logSuccess) {
    Instant now = Instant.now();
    String fileName = FILE_STAMP.format(now) + "-" + trigger + ".json";
    Path directory = Path.of(System.getProperty("opentcs.base", "."), "log", childDirectory);
    Path file = directory.resolve(fileName);
    try {
      Files.createDirectories(directory);
      JSON.writerWithDefaultPrettyPrinter().writeValue(file.toFile(), value);
      if (logSuccess) {
        LOG.info("[MAPF capture] wrote {}", file.toAbsolutePath());
      }
      return file.toAbsolutePath();
    } catch (IOException e) {
      LOG.warn("[MAPF capture] failed to write {}: {}", file.toAbsolutePath(), e.toString());
      return null;
    }
  }

  private static String safeName(String trigger) {
    return trigger.replaceAll("[^A-Za-z0-9_-]", "_");
  }

  private static synchronized boolean claimDumpSlot(String trigger, long now, long minGapMs) {
    Long previous = LAST_DUMP_BY_TRIGGER.get(trigger);
    if (previous != null && now - previous < minGapMs) {
      return false;
    }
    LAST_DUMP_BY_TRIGGER.put(trigger, now);
    return true;
  }
}
