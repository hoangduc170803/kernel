package org.opentcs.fms.routing;

import org.opentcs.fms.routing.diag.GuideCounters;

/** The local guidance contract consumed by LaCAM and PIBT. */
public interface Guidance {

  boolean enabled();

  void construct(int[] configuration, int[] noses, int[] order, int solveDepth);

  default void construct(
      int[] configuration, int[] noses, int[] motions, int[] order, int solveDepth) {
    construct(configuration, noses, order, solveDepth);
  }

  int heuristic(int agent, int vertex);

  int[] guidedConfiguration();

  float collisionCostOf(int agent);

  int footprintConflictsOf(int agent);

  GuideCounters counters();

  int window();

  int[][] currentGuidePaths();

  void setGuidePaths(int[][] paths);
}
