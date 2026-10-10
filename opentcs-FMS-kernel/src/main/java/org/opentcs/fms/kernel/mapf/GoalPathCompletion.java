package org.opentcs.fms.kernel.mapf;

import java.util.ArrayList;
import java.util.List;
import org.opentcs.fms.routing.Roadmap;

final class GoalPathCompletion {

  private GoalPathCompletion() {
  }

  static Result complete(Roadmap map, int[][] paths, int[] goals, int[][] distances) {
    List<int[]> completedPaths = new ArrayList<>(paths.length);
    List<Integer> strandedAgents = new ArrayList<>();
    int maxLength = 0;
    for (int agent = 0; agent < paths.length; agent++) {
      int[] completedPath = completePathToGoal(
          map, paths[agent], goals[agent], distances[agent]);
      if (completedPath.length == 0 || completedPath[completedPath.length - 1] != goals[agent]) {
        strandedAgents.add(agent);
      }
      maxLength = Math.max(maxLength, completedPath.length);
      completedPaths.add(completedPath);
    }
    return new Result(alignLengths(completedPaths, maxLength), strandedAgents);
  }

  private static int[] completePathToGoal(Roadmap map, int[] path, int goal, int[] distances) {
    List<Integer> completed = new ArrayList<>(path.length);
    for (int vertex : path) {
      completed.add(vertex);
    }
    if (completed.isEmpty()) {
      completed.add(goal);
    }

    int current = completed.get(completed.size() - 1);
    while (current != goal) {
      int bestNext = -1;
      long bestCost = Long.MAX_VALUE;
      for (int neighbor : map.neighborsOf(current)) {
        long costThroughNeighbor = (long) map.weightOf(neighbor) + distances[neighbor];
        if (costThroughNeighbor < bestCost) {
          bestCost = costThroughNeighbor;
          bestNext = neighbor;
        }
      }
      if (bestNext == -1 || distances[bestNext] >= distances[current]) {
        break;
      }
      completed.add(bestNext);
      current = bestNext;
    }
    return toArray(completed);
  }

  private static int[][] alignLengths(List<int[]> paths, int maxLength) {
    int[][] aligned = new int[paths.size()][];
    for (int agent = 0; agent < paths.size(); agent++) {
      int[] path = paths.get(agent);
      int[] alignedPath = new int[maxLength];
      for (int time = 0; time < maxLength; time++) {
        alignedPath[time] = time < path.length ? path[time] : path[path.length - 1];
      }
      aligned[agent] = alignedPath;
    }
    return aligned;
  }

  private static int[] toArray(List<Integer> path) {
    int[] result = new int[path.size()];
    for (int index = 0; index < path.size(); index++) {
      result[index] = path.get(index);
    }
    return result;
  }

  record Result(int[][] paths, List<Integer> strandedAgents) {

    Result {
      strandedAgents = List.copyOf(strandedAgents);
    }
  }
}
