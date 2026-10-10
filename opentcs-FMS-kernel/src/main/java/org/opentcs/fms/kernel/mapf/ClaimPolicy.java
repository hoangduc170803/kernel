package org.opentcs.fms.kernel.mapf;

import java.util.Map;
import java.util.Set;

public record ClaimPolicy(
    Map<String, Long> rank,
    Set<String> retreating,
    Set<String> droppingOff,
    Set<String> atGoal,
    Set<String> dropOffLeg,
    Map<String, String> body,
    Map<String, String> retreatGoal) {

  public static final ClaimPolicy DEFAULT =
      new ClaimPolicy(Map.of(), Set.of(), Set.of(), Set.of(), Set.of(), Map.of(), Map.of());

  public static ClaimPolicy byRank(Map<String, Long> rank) {
    return new ClaimPolicy(rank, Set.of(), Set.of(), Set.of(), Set.of(), Map.of(), Map.of());
  }

  public boolean isRetreating(String vehicle) {
    return retreating.contains(vehicle);
  }

  public boolean isDroppingOff(String vehicle) {
    return droppingOff.contains(vehicle);
  }

  public boolean isAtGoal(String vehicle) {
    return atGoal.contains(vehicle);
  }

  public boolean isOnDropOffLeg(String vehicle) {
    return dropOffLeg.contains(vehicle);
  }

  public String bodyOf(String vehicle) {
    return body.get(vehicle);
  }

  public Map<String, String> retreatGoals() {
    return retreatGoal;
  }

  public String retreatGoalOf(String vehicle) {
    return retreatGoal.get(vehicle);
  }
}
