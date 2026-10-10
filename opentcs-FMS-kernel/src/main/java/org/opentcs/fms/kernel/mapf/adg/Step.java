package org.opentcs.fms.kernel.mapf.adg;

public record Step(String vehicle, int step) {

  @Override
  public String toString() {
    return vehicle + "#" + step;
  }
}
