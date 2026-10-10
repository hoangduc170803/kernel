package org.opentcs.fms.kernel.mapf;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import org.opentcs.fms.routing.Roadmap;

final class GoalClaimer {

  private static final org.slf4j.Logger logger =
      org.slf4j.LoggerFactory.getLogger(GoalClaimer.class);

  private static final int UNREACHABLE = Integer.MAX_VALUE;

  private final Roadmap roadmap;

  GoalClaimer(Roadmap roadmap) {
    this.roadmap = roadmap;
  }

  GoalClaims claim(
      Roadmap planningRoadmap, List<String> agents, Map<String, String> goals, int[] starts) {
    int n = agents.size();
    int[] goalVertices = new int[n];
    List<String> stranded = new ArrayList<>();

    for (int a = 0; a < n; a++) {
      String vehicle = agents.get(a);
      String goal = goals.get(vehicle);
      int desired = (goal != null && roadmap.has(goal)) ? roadmap.idOf(goal) : starts[a];

      if (desired != starts[a] && !reachable(planningRoadmap, starts[a], desired)) {
        goalVertices[a] = starts[a];
        stranded.add(vehicle + "->" + roadmap.nameOf(desired));
        continue;
      }
      goalVertices[a] = desired;
    }

    GoalClaims claims = new GoalClaims(goalVertices, stranded);
    logStranded(claims);
    return claims;
  }

  private static void logStranded(GoalClaims claims) {
    if (claims.stranded().isEmpty()) {
      return;
    }
    logger.warn(
        "[MAPF goal] {} agent(s) cannot reach the goal WES asked for and are held at their current"
            + " point: {}. Their cached route ends there, NOT at that goal, so FMSRouter cannot use"
            + " it as a coordinated prefix. Goals come from WES alone — the planner never"
            + " substitutes one, so this means the way in is walled off by a standing vehicle.",
        claims.stranded().size(), claims.stranded());
  }

  static boolean reachable(Roadmap map, int start, int goal) {
    return hopsFrom(map, start)[goal] != UNREACHABLE;
  }

  private static int[] hopsFrom(Roadmap map, int start) {
    int[] hops = new int[map.size()];
    Arrays.fill(hops, UNREACHABLE);
    int[] queue = new int[map.size()];
    int tail = 0;
    hops[start] = 0;
    queue[tail++] = start;
    for (int head = 0; head < tail; head++) {
      int v = queue[head];
      for (int next : map.neighborsOf(v)) {
        if (hops[next] == UNREACHABLE) {
          hops[next] = hops[v] + 1;
          queue[tail++] = next;
        }
      }
    }
    return hops;
  }
}
