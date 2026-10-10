package org.opentcs.fms.kernel.mapf;

import java.util.EnumMap;
import java.util.Map;

final class WarmStartMemoryLogger {

  private static final org.slf4j.Logger logger =
      org.slf4j.LoggerFactory.getLogger(WarmStartMemoryLogger.class);

  private static final int NO_VERTEX = MapfPlanner.NO_VERTEX;

  private enum Origin {
    CARRIED_OVER,
    GOAL_CHANGED,
    DROPPED_THOUGH_GOAL_UNCHANGED,
    NEVER_PLANNED
  }

  private final Map<String, Integer> lastPlanGoals;
  private final Map<String, Integer> goalsOfPlansDroppedWhenAVehicleLeftTheSolve;
  private final EnumMap<Origin, Integer> census = new EnumMap<>(Origin.class);

  WarmStartMemoryLogger(
      Map<String, Integer> lastPlanGoals,
      Map<String, Integer> goalsOfPlansDroppedWhenAVehicleLeftTheSolve) {
    this.lastPlanGoals = lastPlanGoals;
    this.goalsOfPlansDroppedWhenAVehicleLeftTheSolve = goalsOfPlansDroppedWhenAVehicleLeftTheSolve;
  }

  void record(String vehicle, int goal) {
    census.merge(originOf(vehicle, goal), 1, Integer::sum);
  }

  void logAndReset(int agents) {
    logger.debug(
        "[MAPF seed] agents={} carriedOver={} droppedGoalChanged={}"
            + " droppedThoughGoalUnchanged={} neverPlanned={}",
        agents,
        countOf(Origin.CARRIED_OVER),
        countOf(Origin.GOAL_CHANGED),
        countOf(Origin.DROPPED_THOUGH_GOAL_UNCHANGED),
        countOf(Origin.NEVER_PLANNED));
    census.clear();
  }

  private Origin originOf(String vehicle, int goal) {
    Integer plannedGoal = lastPlanGoals.get(vehicle);
    if (plannedGoal != null) {
      return plannedGoal == goal ? Origin.CARRIED_OVER : Origin.GOAL_CHANGED;
    }
    return goalsOfPlansDroppedWhenAVehicleLeftTheSolve.getOrDefault(vehicle, NO_VERTEX) == goal
        ? Origin.DROPPED_THOUGH_GOAL_UNCHANGED
        : Origin.NEVER_PLANNED;
  }

  private int countOf(Origin origin) {
    return census.getOrDefault(origin, 0);
  }
}
