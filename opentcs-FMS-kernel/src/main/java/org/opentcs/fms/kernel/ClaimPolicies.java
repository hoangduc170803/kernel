package org.opentcs.fms.kernel;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function;

import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.DriveOrder;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.fms.kernel.mapf.ClaimPolicy;
import org.opentcs.fms.kernel.scheduling.RetreatHolds;

public final class ClaimPolicies {

  private ClaimPolicies() {
  }

  public static long claimRankOf(TransportOrder order) {
    return order.getCreationTime().toEpochMilli();
  }

  public static ClaimPolicy of(
      Collection<Vehicle> vehicles, Function<Vehicle, Optional<TransportOrder>> orderOf) {
    return of(vehicles, orderOf, Set.of());
  }

  public static ClaimPolicy of(
      Collection<Vehicle> vehicles,
      Function<Vehicle, Optional<TransportOrder>> orderOf,
      Set<String> atGoal) {
    Map<String, Long> rank = new HashMap<>();
    Set<String> retreating = new HashSet<>();
    Set<String> droppingOff = new HashSet<>();
    Set<String> dropOffLeg = new HashSet<>();
    Map<String, String> body = new HashMap<>();
    Map<String, String> retreatGoals = new HashMap<>();
    for (Vehicle vehicle : vehicles) {
      var standingOn = vehicle.getCurrentPosition();
      if (standingOn != null) {
        body.put(vehicle.getName(), standingOn.getName());
      }
      orderOf.apply(vehicle).ifPresent(order -> {
        rank.put(vehicle.getName(), claimRankOf(order));
        if (RetreatHolds.isRetreating(order)) {
          retreating.add(vehicle.getName());
          String retreatGoal = retreatGoalOf(order);
          if (retreatGoal != null) {
            retreatGoals.put(vehicle.getName(), retreatGoal);
          }
        }
        if (RetreatHolds.isDropOffLeg(order)) {
          dropOffLeg.add(vehicle.getName());
        }
        if (RetreatHolds.isDropOffLeg(order) || RetreatHolds.isApproachLeg(order)) {
          droppingOff.add(vehicle.getName());
        }
      });
    }
    return new ClaimPolicy(
        rank, Set.copyOf(retreating), Set.copyOf(droppingOff), Set.copyOf(atGoal),
        Set.copyOf(dropOffLeg), Map.copyOf(body), Map.copyOf(retreatGoals));
  }

  private static String retreatGoalOf(TransportOrder order) {
    DriveOrder current = order.getCurrentDriveOrder();
    if (current == null || current.getDestination() == null
        || current.getDestination().getDestination() == null) {
      return null;
    }
    return current.getDestination().getDestination().getName();
  }
}
