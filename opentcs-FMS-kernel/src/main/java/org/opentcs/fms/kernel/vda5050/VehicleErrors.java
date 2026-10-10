package org.opentcs.fms.kernel.vda5050;

import static org.opentcs.commadapter.vehicle.vda5050.v2_0.ObjectProperties.PROPKEY_VEHICLE_ERRORS_FATAL;

import java.util.LinkedHashSet;
import java.util.Set;

import org.opentcs.data.model.Vehicle;

public final class VehicleErrors {

  public static final String LOST_NAVIGATION = "adapterLostNavigation";

  private VehicleErrors() {
  }

  public static Set<String> fatalErrorTypes(Vehicle vehicle) {
    String joined = vehicle.getProperty(PROPKEY_VEHICLE_ERRORS_FATAL);
    if (joined == null || joined.isBlank()) {
      return Set.of();
    }
    Set<String> errorTypes = new LinkedHashSet<>();
    for (String errorType : joined.split(",")) {
      String trimmed = errorType.trim();
      if (!trimmed.isEmpty()) {
        errorTypes.add(trimmed);
      }
    }
    return errorTypes;
  }

  public static boolean isOnlyLostNavigation(Vehicle vehicle) {
    return fatalErrorTypes(vehicle).equals(Set.of(LOST_NAVIGATION));
  }
}
