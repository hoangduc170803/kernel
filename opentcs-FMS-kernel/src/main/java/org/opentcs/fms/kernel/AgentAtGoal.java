package org.opentcs.fms.kernel;

import java.util.Collection;
import java.util.Collections;
import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;
import java.util.function.Function;
import java.util.function.Predicate;

import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.fms.kernel.scheduling.RetreatHolds;

public final class AgentAtGoal {

  public enum Arrival {
    CHARGING,
    RETREATED,
    PARKED
  }

  private AgentAtGoal() {
  }

  public static Map<String, Arrival> among(
      Collection<Vehicle> vehicles,
      Function<Vehicle, Optional<TransportOrder>> orderOf,
      Map<String, String> positions,
      Map<String, String> goals,
      Predicate<String> restingPoint) {
    Map<String, Arrival> arrived = new TreeMap<>();
    for (Vehicle vehicle : vehicles) {
      String name = vehicle.getName();
      Arrival arrival = reached(
          vehicle, orderOf.apply(vehicle), positions.get(name), goals.get(name), restingPoint);
      if (arrival != null) {
        arrived.put(name, arrival);
      }
    }
    return Collections.unmodifiableMap(arrived);
  }

  public static Arrival reached(
      Vehicle vehicle,
      Optional<TransportOrder> order,
      String position,
      String goal,
      Predicate<String> restingPoint) {
    if (vehicle.hasState(Vehicle.State.CHARGING)) {
      return Arrival.CHARGING;
    }
    if (position == null || !position.equals(goal)) {
      return null;
    }
    if (order.map(RetreatHolds::isRetreating).orElse(false)) {
      return Arrival.RETREATED;
    }
    return restingPoint.test(position) ? Arrival.PARKED : null;
  }
}
