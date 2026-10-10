package org.opentcs.fms.routing;

import java.util.ArrayList;
import java.util.List;

/** Utilities for working with paths. */
public final class PathUtils {

  private PathUtils() {
  }

  /** Returns defensive copies of the agent-indexed paths. */
  public static int[][] copyOf(int[][] paths, int agents) {
    int[][] copy = new int[agents][];
    for (int agent = 0; agent < agents; agent++) {
      copy[agent] = paths == null || agent >= paths.length || paths[agent] == null
          ? new int[0]
          : paths[agent].clone();
    }
    return copy;
  }

  /**
   * Compresses a solver plan into an openTCS route by removing repeated points that represent
   * wait actions.
   */
  public static List<String> compress(List<String> path) {
    List<String> route = new ArrayList<>();
    for (String point : path) {
      if (route.isEmpty() || !route.get(route.size() - 1).equals(point)) {
        route.add(point);
      }
    }
    return route;
  }

  /** Compresses a solver path into a route by removing repeated vertices that represent waits. */
  public static int[] compress(int[] path) {
    List<Integer> route = new ArrayList<>();
    for (int vertex : path) {
      if (route.isEmpty() || route.get(route.size() - 1) != vertex) {
        route.add(vertex);
      }
    }
    int[] compressed = new int[route.size()];
    for (int i = 0; i < compressed.length; i++) {
      compressed[i] = route.get(i);
    }
    return compressed;
  }
}
