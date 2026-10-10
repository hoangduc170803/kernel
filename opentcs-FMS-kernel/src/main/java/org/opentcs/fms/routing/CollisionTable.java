package org.opentcs.fms.routing;

import java.util.ArrayList;
import java.util.List;

public final class CollisionTable {

  public static final int NO_VERTEX = -1;

  private static final int NO_AGENT = -1;

  private final List<List<List<Integer>>> body;
  private final List<List<Integer>> arrivalTimes;
  private final boolean noUseCollisionCount;
  private final boolean noUseGoalOccupation;

  private int collisionCount;

  public CollisionTable(
      int vertexCount, boolean noUseCollisionCount, boolean noUseGoalOccupation) {
    this.body = new ArrayList<>(vertexCount);
    this.arrivalTimes = new ArrayList<>(vertexCount);
    for (int vertex = 0; vertex < vertexCount; vertex++) {
      body.add(new ArrayList<>());
      arrivalTimes.add(new ArrayList<>());
    }
    this.noUseCollisionCount = noUseCollisionCount;
    this.noUseGoalOccupation = noUseGoalOccupation;
    this.collisionCount = 0;
  }

  public int collisionCount() {
    return collisionCount;
  }

  public int collisionCost(int from, int to, int timeFrom) {
    return collisionCost(from, to, timeFrom, NO_AGENT);
  }

  public int collisionCost(int from, int to, int timeFrom, int excludeAgent) {
    if (from == NO_VERTEX || to == NO_VERTEX) {
      return 0;
    }
    int timeTo = timeFrom + 1;
    List<List<Integer>> leavingTimeline = body.get(from);
    List<List<Integer>> arrivingTimeline = body.get(to);
    int collisions = 0;

    if (timeTo < arrivingTimeline.size()) {
      for (int agent : arrivingTimeline.get(timeTo)) {
        if (agent != excludeAgent) {
          collisions++;
        }
      }
    }

    if (timeTo < leavingTimeline.size() && timeFrom < arrivingTimeline.size()) {
      for (int comingBack : leavingTimeline.get(timeTo)) {
        for (int headingOut : arrivingTimeline.get(timeFrom)) {
          if (comingBack == headingOut && comingBack != excludeAgent) {
            collisions++;
          }
        }
      }
    }

    if (from != to && timeFrom < arrivingTimeline.size()) {
      for (int occupant : arrivingTimeline.get(timeFrom)) {
        if (occupant != excludeAgent && !occupies(arrivingTimeline, timeTo, occupant)) {
          collisions++;
        }
      }
    }

    if (!noUseGoalOccupation) {
      for (int settledAt : arrivalTimes.get(to)) {
        if (timeTo > settledAt) {
          collisions++;
        }
      }
    }
    return collisions;
  }

  private static boolean occupies(List<List<Integer>> timeline, int time, int agent) {
    if (time >= timeline.size()) {
      return false;
    }
    for (int occupant : timeline.get(time)) {
      if (occupant == agent) {
        return true;
      }
    }
    return false;
  }

  List<Integer> occupantsAt(int vertex, int time) {
    if (vertex == NO_VERTEX || time < 0) {
      return List.of();
    }
    List<List<Integer>> timeline = body.get(vertex);
    return time < timeline.size() ? List.copyOf(timeline.get(time)) : List.of();
  }

  public void enrollPath(int agent, int[] path) {
    if (path.length == 0 || path[0] == NO_VERTEX) {
      return;
    }
    int lastTime = path.length - 1;
    for (int t = 0; t <= lastTime; t++) {
      if (path[t] == NO_VERTEX) {
        continue;
      }
      if (t > 0 && path[t - 1] != NO_VERTEX && !noUseCollisionCount) {
        collisionCount += collisionCost(path[t - 1], path[t], t - 1);
      }
      occupy(path[t], t, agent);
    }

    if (!noUseGoalOccupation && path[lastTime] != NO_VERTEX) {
      arrivalTimes.get(path[lastTime]).add(lastTime);
      if (!noUseCollisionCount) {
        collisionCount += occupantsAfter(path[lastTime], lastTime);
      }
    }
  }

  public void clearPath(int agent, int[] path) {
    if (path.length == 0 || path[0] == NO_VERTEX) {
      return;
    }
    int lastTime = path.length - 1;
    for (int t = 0; t <= lastTime; t++) {
      int vertex = path[t];
      if (vertex == NO_VERTEX) {
        continue;
      }
      List<List<Integer>> timeline = body.get(vertex);
      if (t >= timeline.size()) {
        continue;
      }
      timeline.get(t).remove(Integer.valueOf(agent));
      if (t > 0 && path[t - 1] != NO_VERTEX && !noUseCollisionCount) {
        collisionCount -= collisionCost(path[t - 1], path[t], t - 1);
      }
    }

    if (!noUseGoalOccupation && path[lastTime] != NO_VERTEX) {
      arrivalTimes.get(path[lastTime]).remove(Integer.valueOf(lastTime));
      if (!noUseCollisionCount) {
        collisionCount -= occupantsAfter(path[lastTime], lastTime);
      }
    }
  }

  private void occupy(int vertex, int time, int agent) {
    List<List<Integer>> timeline = body.get(vertex);
    while (timeline.size() <= time) {
      timeline.add(new ArrayList<>());
    }
    timeline.get(time).add(agent);
  }

  private int occupantsAfter(int vertex, int time) {
    List<List<Integer>> timeline = body.get(vertex);
    int count = 0;
    for (int t = time + 1; t < timeline.size(); t++) {
      count += timeline.get(t).size();
    }
    return count;
  }
}
