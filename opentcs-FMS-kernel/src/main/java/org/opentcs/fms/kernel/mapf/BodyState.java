package org.opentcs.fms.kernel.mapf;

import java.util.Map;
import java.util.Set;

import org.opentcs.fms.routing.Roadmap;

public record BodyState(
    Map<String, Integer> axisByVehicle, Set<String> loadedVehicles) {

  public static final BodyState UNKNOWN = new BodyState(Map.of(), Set.of());

  public int axisOf(String vehicle) {
    return axisByVehicle.getOrDefault(vehicle, Roadmap.DIR_NONE);
  }

  public boolean loaded(String vehicle) {
    return loadedVehicles.contains(vehicle);
  }

}
