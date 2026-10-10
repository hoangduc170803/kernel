package org.opentcs.fms.kernel.mapf;

import java.util.List;
import java.util.Map;
import java.util.Set;
import org.opentcs.fms.routing.pibt.Footprint;

final class ReplanPolicy {

  private static final long MIN_SOLVE_GAP_MS = Long.getLong("fms.mapf.minSolveGapMs", 1_000L);
  private static final long COMMIT_MS = Long.getLong("fms.mapf.commitMs", 4_000L);

  private volatile Map<String, String> lastGoals = Map.of();
  private volatile Set<String> lastImmobile = Set.of();
  private volatile List<Footprint.Obstacle> lastObstacles = List.of();
  private volatile List<Footprint.Obstacle> observedObstacles = List.of();
  private volatile long lastSolveEndedAtMs = Long.MIN_VALUE;
  private volatile boolean solveOwedFromThrottle;

  boolean shouldSolve(Map<String, String> positions, Map<String, String> goals,
      Set<String> immobile, Map<String, MapfCoordinator.PlannedRoute> currentPlan, long now) {
    long sinceSolve = now - lastSolveEndedAtMs;
    boolean tooSoon = !currentPlan.isEmpty() && sinceSolve < MIN_SOLVE_GAP_MS;
    boolean changed = shouldReplan(positions, goals, immobile, currentPlan);
    if (tooSoon || !(overdueAt(now, currentPlan) || changed)) {
      solveOwedFromThrottle |= tooSoon && changed;
      return false;
    }
    solveOwedFromThrottle = false;
    return true;
  }

  void recordSolveEnded(long now) {
    lastSolveEndedAtMs = now;
  }

  void recordSolved(Map<String, String> goals, Set<String> immobile) {
    lastGoals = Map.copyOf(goals);
    lastImmobile = Set.copyOf(immobile);
  }

  void recordSolved(
      Map<String, String> goals, Set<String> immobile, List<Footprint.Obstacle> obstacles) {
    recordSolved(goals, immobile);
    lastObstacles = List.copyOf(obstacles);
  }

  void observeObstacles(List<Footprint.Obstacle> obstacles) {
    observedObstacles = List.copyOf(obstacles);
  }

  boolean overdueAt(long now, Map<String, MapfCoordinator.PlannedRoute> currentPlan) {
    if (currentPlan.isEmpty()) {
      return false;
    }
    long sinceSolve = now - lastSolveEndedAtMs;
    return sinceSolve >= COMMIT_MS
        || (solveOwedFromThrottle && sinceSolve >= MIN_SOLVE_GAP_MS);
  }

  private boolean shouldReplan(Map<String, String> positions, Map<String, String> goals,
      Set<String> immobile, Map<String, MapfCoordinator.PlannedRoute> currentPlan) {
    if (currentPlan.isEmpty()) {
      return true;
    }
    if (!goals.equals(lastGoals)) {
      return true;
    }
    if (!immobile.equals(lastImmobile)) {
      return true;
    }
    if (!observedObstacles.equals(lastObstacles)) {
      return true;
    }
    for (Map.Entry<String, String> e : positions.entrySet()) {
      String goal = goals.get(e.getKey());
      if (goal == null || goal.equals(e.getValue())) {
        continue;
      }
      MapfCoordinator.PlannedRoute planned = currentPlan.get(e.getKey());
      if (planned == null || planned.points().indexOf(e.getValue()) < 0) {
        return true;
      }
    }
    return false;
  }
}
