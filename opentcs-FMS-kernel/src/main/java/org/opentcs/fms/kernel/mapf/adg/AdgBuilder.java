package org.opentcs.fms.kernel.mapf.adg;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.opentcs.fms.kernel.mapf.BodyState;
import org.opentcs.fms.routing.PathUtils;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.pibt.Footprint;
import org.opentcs.fms.routing.pibt.Footprint.BodyPose;

public final class AdgBuilder {

  private AdgBuilder() {
  }

  private record Track(
      String vehicle, List<String> route, int[] arrivalTick, BodyPose[] poses,
      int coordinatedStep) {

    boolean hasGeometry() {
      return poses != null;
    }
  }

  public static Adg build(
      Map<String, List<String>> spaceTimePaths, int coordinatedTicks, Roadmap roadmap,
      Footprint footprint, BodyState bodyState) {
    Map<String, List<String>> routes = routesOf(spaceTimePaths);
    Footprint geometry = roadmap != null && footprint != null && Footprint.enabled() ? footprint : null;
    List<Track> tracks = new ArrayList<>();
    for (Map.Entry<String, List<String>> entry : spaceTimePaths.entrySet()) {
      Track track = buildTrack(
          entry.getKey(), entry.getValue(), coordinatedTicks, roadmap, geometry, bodyState);
      if (track != null) {
        tracks.add(track);
      }
    }
    tracks.sort(Comparator.comparing(Track::vehicle));
    Map<String, Integer> coordinatedSteps = new LinkedHashMap<>();
    for (Track track : tracks) {
      coordinatedSteps.put(track.vehicle(), track.coordinatedStep());
    }
    return Adg.of(dependenciesOf(tracks, geometry), routes, coordinatedSteps);
  }

  private static Track buildTrack(
      String vehicle, List<String> path, int coordinatedTicks, Roadmap roadmap,
      Footprint footprint, BodyState bodyState) {
    List<String> route = PathUtils.compress(path);
    if (route.isEmpty()) {
      return null;
    }
    int[] arrivalTick = computeArrivalTicksOf(path, route.size());
    return new Track(vehicle, List.copyOf(route), arrivalTick,
        computeBodyPosesOf(vehicle, route, roadmap, footprint, bodyState),
        lastStepArrivingBefore(arrivalTick, Math.min(coordinatedTicks, path.size())));
  }

  private static int lastStepArrivingBefore(int[] arrivalTick, int tick) {
    int step = 0;
    while (step + 1 < arrivalTick.length && arrivalTick[step + 1] < tick) {
      step++;
    }
    return step;
  }

  private static BodyPose[] computeBodyPosesOf(
      String vehicle, List<String> route, Roadmap roadmap, Footprint footprint,
      BodyState bodyState) {
    if (footprint == null) {
      return null;
    }
    int startingNose = bodyState.axisOf(vehicle);
    if (startingNose == Roadmap.DIR_NONE) {
      return null;
    }
    int[] cells = new int[route.size()];
    for (int step = 0; step < cells.length; step++) {
      Integer vertex = roadmap.idOf(route.get(step));
      if (vertex == null) {
        return null;
      }
      cells[step] = vertex;
    }
    int[] noses = new int[cells.length];
    noses[0] = startingNose;
    for (int step = 1; step < cells.length; step++) {
      noses[step] = footprint.noseAfter(noses[step - 1], cells[step - 1], cells[step]);
    }
    boolean loaded = bodyState.loaded(vehicle);
    BodyPose[] poses = new BodyPose[cells.length];
    for (int step = 0; step < cells.length; step++) {
      boolean turning = step + 1 < cells.length
          && footprint.isRotation(noses[step], cells[step], cells[step + 1]);
      poses[step] = new BodyPose(cells[step], noses[step], loaded, turning);
    }
    return poses;
  }

  private static int[] computeArrivalTicksOf(List<String> path, int steps) {
    int[] arrivalTick = new int[steps];
    int step = 0;
    for (int tick = 0; tick < path.size(); tick++) {
      if (tick == 0 || !path.get(tick).equals(path.get(tick - 1))) {
        arrivalTick[step++] = tick;
      }
    }
    return arrivalTick;
  }

  private static Map<Step, List<Step>> dependenciesOf(
      List<Track> tracks, Footprint footprint) {
    Map<Step, List<Step>> dependencies = new LinkedHashMap<>();
    for (Track waiter : tracks) {
      for (int step = 0; step < waiter.route().size(); step++) {
        List<Step> predecessors = predecessorsOf(waiter, step, tracks, footprint);
        if (!predecessors.isEmpty()) {
          dependencies.put(new Step(waiter.vehicle(), step), predecessors);
        }
      }
    }
    return dependencies;
  }

  private static List<Step> predecessorsOf(
      Track waiter, int step, List<Track> tracks, Footprint footprint) {
    List<Step> predecessors = new ArrayList<>();
    for (Track blocker : tracks) {
      if (blocker == waiter) {
        continue;
      }
      int blockingStep = findLatestPredecessorStep(waiter, step, blocker, footprint);
      if (blockingStep >= 0) {
        predecessors.add(new Step(blocker.vehicle(), blockingStep));
      }
    }
    return List.copyOf(predecessors);
  }

  private static int findLatestPredecessorStep(
      Track waiter, int step, Track blocker, Footprint footprint) {
    int arrival = waiter.arrivalTick()[step];
    for (int candidate = blocker.coordinatedStep(); candidate >= 0; candidate--) {
      if (blocker.arrivalTick()[candidate] >= arrival) {
        continue;
      }
      if (!positionsConflict(waiter, step, blocker, candidate, footprint)) {
        continue;
      }
      if (departureOf(blocker, candidate) > arrival) {
        continue;
      }
      return candidate;
    }
    return -1;
  }

  private static int departureOf(Track blocker, int step) {
    return step + 1 <= blocker.coordinatedStep()
        ? blocker.arrivalTick()[step + 1]
        : Integer.MAX_VALUE;
  }

  private static boolean positionsConflict(
      Track waiter, int step, Track blocker, int candidate, Footprint footprint) {
    if (waiter.route().get(step).equals(blocker.route().get(candidate))) {
      return true;
    }
    return footprint != null && waiter.hasGeometry() && blocker.hasGeometry()
        && footprint.bodiesClash(waiter.poses()[step], blocker.poses()[candidate]);
  }

  private static Map<String, List<String>> routesOf(Map<String, List<String>> spaceTimePaths) {
    Map<String, List<String>> routes = new HashMap<>();
    spaceTimePaths.forEach((vehicle, path) -> {
      List<String> route = PathUtils.compress(path);
      if (!route.isEmpty()) {
        routes.put(vehicle, List.copyOf(route));
      }
    });
    return Map.copyOf(routes);
  }

}
