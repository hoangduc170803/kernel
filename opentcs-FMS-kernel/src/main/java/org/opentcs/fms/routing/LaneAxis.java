package org.opentcs.fms.routing;

import java.util.ArrayList;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public final class LaneAxis {

  public static final String LAYOUT_DIRECTION_PROPERTY = "direction";

  private LaneAxis() {
  }

  public static int of(
      List<Roadmap.Point> points, List<Roadmap.Path> paths, boolean declaredByTheLayout) {
    if (declaredByTheLayout) {
      return Roadmap.AXIS_Y;
    }
    int running = axisWithTheLongestChain(points, paths);
    return running == Roadmap.AXIS_X ? Roadmap.AXIS_Y : Roadmap.AXIS_X;
  }

  private static int axisWithTheLongestChain(
      List<Roadmap.Point> points, List<Roadmap.Path> paths) {
    double bestRatio = -1;
    int best = Roadmap.AXIS_X;
    for (int axis : new int[] { Roadmap.AXIS_X, Roadmap.AXIS_Y }) {
      double extent = extentAlong(points, axis);
      if (extent == 0) {
        continue;
      }
      double longest = 0;
      for (List<Roadmap.Point> chain : chainsAlong(points, paths, axis)) {
        longest = Math.max(longest, spanAlong(chain, axis));
      }
      double ratio = longest / extent;
      if (ratio > bestRatio) {
        bestRatio = ratio;
        best = axis;
      }
    }
    return best;
  }

  private static List<List<Roadmap.Point>> chainsAlong(
      List<Roadmap.Point> points, List<Roadmap.Path> paths, int axis) {
    Map<String, Roadmap.Point> byName = new HashMap<>();
    for (Roadmap.Point point : points) {
      byName.put(point.name(), point);
    }
    Map<String, Set<String>> neighbours = new HashMap<>();
    for (Roadmap.Path path : paths) {
      Roadmap.Point src = byName.get(path.src());
      Roadmap.Point dst = byName.get(path.dst());
      if (src == null || dst == null || !runsAlong(axis, src, dst)) {
        continue;
      }
      neighbours.computeIfAbsent(src.name(), k -> new HashSet<>()).add(dst.name());
      neighbours.computeIfAbsent(dst.name(), k -> new HashSet<>()).add(src.name());
    }

    Set<String> seen = new HashSet<>();
    List<List<Roadmap.Point>> chains = new ArrayList<>();
    for (String start : neighbours.keySet()) {
      if (!seen.add(start)) {
        continue;
      }
      List<Roadmap.Point> chain = new ArrayList<>();
      Deque<String> stack = new ArrayDeque<>();
      stack.push(start);
      while (!stack.isEmpty()) {
        String here = stack.pop();
        Roadmap.Point point = byName.get(here);
        if (point != null) {
          chain.add(point);
        }
        for (String next : neighbours.getOrDefault(here, Set.of())) {
          if (seen.add(next)) {
            stack.push(next);
          }
        }
      }
      chains.add(chain);
    }
    return chains;
  }

  private static boolean runsAlong(int axis, Roadmap.Point from, Roadmap.Point to) {
    double dx = Math.abs(to.x() - from.x());
    double dy = Math.abs(to.y() - from.y());
    return axis == Roadmap.AXIS_X ? dx > dy : dy > dx;
  }

  private static double coordinateAlong(int axis, Roadmap.Point point) {
    return axis == Roadmap.AXIS_X ? point.x() : point.y();
  }

  private static double extentAlong(List<Roadmap.Point> points, int axis) {
    if (points.isEmpty()) {
      return 0;
    }
    double lowest = Double.MAX_VALUE;
    double highest = -Double.MAX_VALUE;
    for (Roadmap.Point point : points) {
      double value = coordinateAlong(axis, point);
      lowest = Math.min(lowest, value);
      highest = Math.max(highest, value);
    }
    return highest - lowest;
  }

  private static double spanAlong(List<Roadmap.Point> chain, int axis) {
    double lowest = Double.MAX_VALUE;
    double highest = -Double.MAX_VALUE;
    for (Roadmap.Point point : chain) {
      double value = coordinateAlong(axis, point);
      lowest = Math.min(lowest, value);
      highest = Math.max(highest, value);
    }
    return chain.isEmpty() ? 0 : highest - lowest;
  }
}
