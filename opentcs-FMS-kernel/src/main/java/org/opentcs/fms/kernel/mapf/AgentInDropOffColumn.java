package org.opentcs.fms.kernel.mapf;

import java.util.HashSet;
import java.util.Set;

import org.opentcs.fms.routing.Roadmap;

final class AgentInDropOffColumn {

  private final Set<String> committed = new HashSet<>();

  boolean test(String vehicle, ClaimPolicy claimPolicy, Roadmap map, int body, int goal) {
    if (!claimPolicy.isOnDropOffLeg(vehicle) || !map.isDropOffSlot(body)) {
      committed.remove(vehicle);
      return false;
    }
    if (committed.contains(vehicle) || map.sharesLaneWith(body, goal)) {
      committed.add(vehicle);
      return true;
    }
    return false;
  }

  int bodyVertexOf(String vehicle, ClaimPolicy claimPolicy, Roadmap map, int frontier) {
    String standingOn = claimPolicy.bodyOf(vehicle);
    return standingOn != null && map.has(standingOn) ? map.idOf(standingOn) : frontier;
  }
}
