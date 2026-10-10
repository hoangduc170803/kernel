package org.opentcs.fms.routing;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.opentcs.fms.routing.pibt.PibtEngine;

/**
 * Test helpers: build a 4-connected grid roadmap and check MAPF invariants on a
 * plan (ported from {@code routing/__tests__/grid.fixture.ts}). The collision
 * checks here are the load-bearing oracle for the engine port.
 */
final class GridFixture {

  private GridFixture() {
  }

  static String cell(int row, int col) {
    return "r" + row + "c" + col;
  }

  /** A {@code width x height} grid, 4-connected, every edge bidirectional. */
  static Roadmap grid(int width, int height) {
    List<Roadmap.Point> points = new ArrayList<>();
    for (int r = 0; r < height; r++) {
      for (int c = 0; c < width; c++) {
        points.add(new Roadmap.Point(cell(r, c), c, r, 0));
      }
    }
    List<Roadmap.Path> paths = new ArrayList<>();
    for (int r = 0; r < height; r++) {
      for (int c = 0; c < width; c++) {
        if (c + 1 < width) {
          paths.add(new Roadmap.Path(
              cell(r, c) + "->" + cell(r, c + 1), cell(r, c), cell(r, c + 1), 1, true));
        }
        if (r + 1 < height) {
          paths.add(new Roadmap.Path(
              cell(r, c) + "->" + cell(r + 1, c), cell(r, c), cell(r + 1, c), 1, true));
        }
      }
    }
    return new Roadmap(new Roadmap.Model(points, paths));
  }

  static int id(Roadmap roadmap, int row, int col) {
    return roadmap.idOf(cell(row, col));
  }

  /** Vertex- and swap-conflict descriptions across an agent-indexed plan; empty = clean. */
  static List<String> findCollisions(int[][] paths) {
    List<String> issues = new ArrayList<>();
    int n = paths.length;
    if (n == 0) {
      return issues;
    }
    int horizon = paths[0].length;
    for (int a = 0; a < n; a++) {
      horizon = Math.min(horizon, paths[a].length);
    }
    for (int t = 0; t < horizon; t++) {
      Map<Integer, Integer> seen = new HashMap<>();
      for (int a = 0; a < n; a++) {
        Integer prev = seen.put(paths[a][t], a);
        if (prev != null) {
          issues.add("vertex conflict @t=" + t + " v=" + paths[a][t] + " agents " + prev + "," + a);
        }
      }
      if (t + 1 < horizon) {
        for (int a = 0; a < n; a++) {
          for (int b = a + 1; b < n; b++) {
            if (paths[a][t] == paths[b][t + 1]
                && paths[b][t] == paths[a][t + 1]
                && paths[a][t] != paths[a][t + 1]) {
              issues.add("swap conflict @t=" + t + " agents " + a + "," + b);
            }
          }
        }
      }
    }
    return issues;
  }

  /** Every consecutive pair in each path must be equal (wait) or an edge of the graph. */
  static List<String> findIllegalMoves(Roadmap roadmap, int[][] paths) {
    List<String> issues = new ArrayList<>();
    for (int a = 0; a < paths.length; a++) {
      for (int t = 0; t + 1 < paths[a].length; t++) {
        int u = paths[a][t];
        int v = paths[a][t + 1];
        if (u == v) {
          continue;
        }
        boolean edge = false;
        for (int nb : roadmap.neighborsOf(u)) {
          if (nb == v) {
            edge = true;
            break;
          }
        }
        if (!edge) {
          issues.add("illegal move agent " + a + " @t=" + t + " " + u + "->" + v);
        }
      }
    }
    return issues;
  }

  static int[][] rollout(
      PibtEngine pibt, Roadmap roadmap, int[] starts, int[] goals, int window) {
    int n = starts.length;
    List<int[]> configs = new ArrayList<>();
    configs.add(starts.clone());
    int[] current = starts.clone();

    double[] priority = new double[n];
    for (int a = 0; a < n; a++) {
      priority[a] = current[a] == goals[a] ? 0 : 1;
    }

    for (int t = 1; t <= window; t++) {
      int[] next = pibt.step(current, priorityOrder(roadmap, priority, current, goals), List.of());
      if (next == null) {
        next = current.clone();
      }
      for (int a = 0; a < n; a++) {
        priority[a] = next[a] == goals[a] ? 0 : priority[a] + 1;
      }
      configs.add(next);
      current = next;
      if (allAtGoal(current, goals)) {
        break;
      }
    }

    int[][] paths = new int[n][configs.size()];
    for (int t = 0; t < configs.size(); t++) {
      int[] cfg = configs.get(t);
      for (int a = 0; a < n; a++) {
        paths[a][t] = cfg[a];
      }
    }
    return paths;
  }

  private static int[] priorityOrder(
      Roadmap roadmap, double[] priority, int[] current, int[] goals) {
    int n = priority.length;
    Integer[] order = new Integer[n];
    for (int a = 0; a < n; a++) {
      order[a] = a;
    }
    Arrays.sort(order, (a, b) -> {
      if (priority[b] != priority[a]) {
        return Double.compare(priority[b], priority[a]);
      }
      return Integer.compare(
          roadmap.distance(current[b], goals[b]),
          roadmap.distance(current[a], goals[a]));
    });
    int[] out = new int[n];
    for (int a = 0; a < n; a++) {
      out[a] = order[a];
    }
    return out;
  }

  private static boolean allAtGoal(int[] config, int[] goals) {
    for (int a = 0; a < config.length; a++) {
      if (config[a] != goals[a]) {
        return false;
      }
    }
    return true;
  }

  static boolean hasAdgCycle(int[][] paths) {
    int n = paths.length;
    if (n == 0) {
      return false;
    }
    int[] routeLength = new int[n];
    Map<Integer, List<int[]>> arrivals = new HashMap<>();
    for (int a = 0; a < n; a++) {
      int step = -1;
      for (int t = 0; t < paths[a].length; t++) {
        if (t == 0 || paths[a][t] != paths[a][t - 1]) {
          step++;
          arrivals.computeIfAbsent(paths[a][t], k -> new ArrayList<>())
              .add(new int[] { a, step, t });
        }
      }
      routeLength[a] = step + 1;
    }

    Map<Long, List<Long>> adjacency = new HashMap<>();
    for (int a = 0; a < n; a++) {
      for (int k = 0; k + 1 < routeLength[a]; k++) {
        adjacency.computeIfAbsent(adgNode(a, k), x -> new ArrayList<>())
            .add(adgNode(a, k + 1));
      }
    }
    for (List<int[]> visits : arrivals.values()) {
      if (visits.stream().mapToInt(v -> v[0]).distinct().count() < 2) {
        continue;
      }
      List<int[]> ordered = new ArrayList<>(visits);
      ordered.sort(Comparator.<int[]>comparingInt(v -> v[2])
          .thenComparingInt(v -> v[0])
          .thenComparingInt(v -> v[1]));
      for (int i = 0; i + 1 < ordered.size(); i++) {
        int[] leaving = ordered.get(i);
        int[] entering = ordered.get(i + 1);
        if (leaving[0] == entering[0]) {
          continue;
        }
        adjacency.computeIfAbsent(adgNode(leaving[0], leaving[1] + 1), x -> new ArrayList<>())
            .add(adgNode(entering[0], entering[1]));
      }
    }

    Map<Long, Integer> state = new HashMap<>();
    for (Long start : adjacency.keySet()) {
      if (state.getOrDefault(start, 0) == 0 && reachesItself(start, adjacency, state)) {
        return true;
      }
    }
    return false;
  }

  private static long adgNode(int agent, int step) {
    return ((long) agent << 32) | (step & 0xffffffffL);
  }

  private static boolean reachesItself(
      long node, Map<Long, List<Long>> adjacency, Map<Long, Integer> state) {
    state.put(node, 1);
    for (long next : adjacency.getOrDefault(node, List.of())) {
      int seen = state.getOrDefault(next, 0);
      if (seen == 1) {
        return true;
      }
      if (seen == 0 && reachesItself(next, adjacency, state)) {
        return true;
      }
    }
    state.put(node, 2);
    return false;
  }
}
