package org.opentcs.fms.kernel.mapf;

import java.util.ArrayList;
import java.util.List;

final class RetreatGoal {

  private RetreatGoal() {
  }

  static int[][] adjust(List<AgentWindowPlan> plans) {
    List<AgentWindowPlan> adjustedPlans = new ArrayList<>(plans);
    for (int agent = 0; agent < adjustedPlans.size(); agent++) {
      AgentWindowPlan plan = adjustedPlans.get(agent);
      if (!plan.mustStopAtGoal()) {
        continue;
      }

      int firstGoalTime = firstIndexOf(plan.path(), plan.goal());
      if (firstGoalTime < 0 || plan.alreadyStaysAtGoalFrom(firstGoalTime)) {
        continue;
      }

      adjustedPlans.set(agent, plan.holdAtGoalFrom(firstGoalTime));
    }
    return pathsOf(adjustedPlans);
  }

  private static int firstIndexOf(int[] path, int vertex) {
    for (int time = 0; time < path.length; time++) {
      if (path[time] == vertex) {
        return time;
      }
    }
    return -1;
  }

  private static int[][] pathsOf(List<AgentWindowPlan> plans) {
    int[][] paths = new int[plans.size()][];
    for (int agent = 0; agent < plans.size(); agent++) {
      paths[agent] = plans.get(agent).path().clone();
    }
    return paths;
  }
}
