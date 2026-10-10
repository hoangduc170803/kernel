package org.opentcs.fms.kernel;

import java.util.Arrays;
import java.util.Collection;
import java.util.Set;
import java.util.stream.Collectors;

public enum VehicleOperations {
  LOOPBACK("PICK_UP", "DROP_OFF", "CHARGE"),
  VDA5050("liftUp", "liftDown", "startCharging");

  private final String load;
  private final String unload;
  private final String charge;

  VehicleOperations(String load, String unload, String charge) {
    this.load = load;
    this.unload = unload;
    this.charge = charge;
  }

  public String load() {
    return load;
  }

  public String unload() {
    return unload;
  }

  public String charge() {
    return charge;
  }

  public static final Set<String> CHARGE_OPERATIONS = Arrays.stream(values())
      .map(VehicleOperations::charge)
      .collect(Collectors.toUnmodifiableSet());

  public static boolean isCharging(Collection<String> allowedOperations) {
    return allowedOperations.stream().anyMatch(CHARGE_OPERATIONS::contains);
  }

  public static final Set<String> UNLOAD_OPERATIONS = Arrays.stream(values())
      .map(VehicleOperations::unload)
      .collect(Collectors.toUnmodifiableSet());

  public static boolean isDropOff(Collection<String> allowedOperations) {
    return allowedOperations.stream().anyMatch(UNLOAD_OPERATIONS::contains);
  }

  public static final Set<String> LOAD_OPERATIONS = Arrays.stream(values())
      .map(VehicleOperations::load)
      .collect(Collectors.toUnmodifiableSet());

  public static boolean isPickUp(Collection<String> allowedOperations) {
    return allowedOperations.stream().anyMatch(LOAD_OPERATIONS::contains);
  }

  public static boolean carriesCargoAt(String destinationOperation) {
    return destinationOperation != null && UNLOAD_OPERATIONS.contains(destinationOperation);
  }
}
