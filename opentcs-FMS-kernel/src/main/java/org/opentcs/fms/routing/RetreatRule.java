package org.opentcs.fms.routing;

import java.util.Arrays;

final class RetreatRule {

  int[] filterCandidates(
      int[] candidates, int vertex, int goal, int motion, int[] distances) {
    if (distances == null || vertex == goal) {
      return vertex == goal && distances != null ? stayOnly(candidates, vertex) : candidates;
    }

    int currentDistance = distances[vertex];
    if (currentDistance == Roadmap.INF) {
      return candidates;
    }

    if (!hasRetreatStep(candidates, vertex, currentDistance, distances)) {
      return motion == Roadmap.DIR_NONE ? candidates : stayOnly(candidates, vertex);
    }

    return keepRetreatSteps(candidates, vertex, currentDistance, distances);
  }

  private static boolean hasRetreatStep(
      int[] candidates, int vertex, int currentDistance, int[] distances) {
    for (int candidate : candidates) {
      if (candidate != vertex && distances[candidate] < currentDistance) {
        return true;
      }
    }
    return false;
  }

  private static int[] keepRetreatSteps(
      int[] candidates, int vertex, int currentDistance, int[] distances) {
    int[] filtered = new int[candidates.length];
    int count = 0;
    for (int candidate : candidates) {
      if (candidate == vertex || distances[candidate] < currentDistance) {
        filtered[count++] = candidate;
      }
    }
    return count == candidates.length ? candidates : Arrays.copyOf(filtered, count);
  }

  private static int[] stayOnly(int[] candidates, int vertex) {
    for (int candidate : candidates) {
      if (candidate == vertex) {
        return new int[] {vertex};
      }
    }
    return candidates;
  }
}
