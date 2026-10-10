package org.opentcs.fms.kernel.mapf;

record AgentWindowPlan(int[] path, int goal, boolean mustStopAtGoal) {

  AgentWindowPlan {
    path = path.clone();
  }

  boolean alreadyStaysAtGoalFrom(int time) {
    for (int t = time + 1; t < path.length; t++) {
      if (path[t] != goal) {
        return false;
      }
    }
    return true;
  }

  AgentWindowPlan holdAtGoalFrom(int time) {
    int[] heldPath = path.clone();
    for (int t = time + 1; t < heldPath.length; t++) {
      heldPath[t] = goal;
    }
    return new AgentWindowPlan(heldPath, goal, mustStopAtGoal);
  }
}
