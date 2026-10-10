package org.opentcs.fms.kernel.vda5050;

public record AubotSpeeds(double loaded, double unloaded, double turning) {

  public double cruising(boolean vehicleIsLoaded) {
    return vehicleIsLoaded ? loaded : unloaded;
  }
}
