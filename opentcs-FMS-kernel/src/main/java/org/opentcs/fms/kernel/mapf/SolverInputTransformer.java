package org.opentcs.fms.kernel.mapf;

import java.util.Arrays;
import java.util.List;

import org.opentcs.fms.routing.Roadmap;

final class SolverInputTransformer {

  private static final int NO_VERTEX = -1;

  int[] retreatGoals(
      List<String> agents, ClaimPolicy claimPolicy, Roadmap roadmap) {
    int[] result = new int[agents.size()];
    Arrays.fill(result, NO_VERTEX);
    for (int agent = 0; agent < agents.size(); agent++) {
      String vehicle = agents.get(agent);
      String goal = claimPolicy.isRetreating(vehicle)
          ? claimPolicy.retreatGoalOf(vehicle)
          : null;
      if (goal != null && roadmap.has(goal)) {
        result[agent] = roadmap.idOf(goal);
      }
    }
    return result;
  }

}
