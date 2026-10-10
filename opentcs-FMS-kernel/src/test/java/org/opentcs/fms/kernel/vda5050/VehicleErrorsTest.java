package org.opentcs.fms.kernel.vda5050;

import static org.assertj.core.api.Assertions.assertThat;
import static org.opentcs.commadapter.vehicle.vda5050.v2_0.ObjectProperties.PROPKEY_VEHICLE_ERRORS_FATAL;

import org.junit.jupiter.api.Test;
import org.opentcs.data.model.Vehicle;

class VehicleErrorsTest {

  private static Vehicle withFatalErrors(String joined) {
    return new Vehicle("V01").withProperty(PROPKEY_VEHICLE_ERRORS_FATAL, joined);
  }

  @Test
  void vehicleWithoutTheErrorPropertyReportsNoFatalErrors() {
    assertThat(VehicleErrors.fatalErrorTypes(new Vehicle("V01"))).isEmpty();
  }

  @Test
  void emptyErrorPropertyReportsNoFatalErrors() {
    assertThat(VehicleErrors.fatalErrorTypes(withFatalErrors("")))
        .as("the driver publishes an empty string once the vehicle recovers, which must read"
            + " as 'no errors' rather than as one error with an empty name")
        .isEmpty();
  }

  @Test
  void singleErrorTypeIsExtracted() {
    assertThat(VehicleErrors.fatalErrorTypes(withFatalErrors("adapterLostNavigation")))
        .containsExactly("adapterLostNavigation");
  }

  @Test
  void commaJoinedErrorTypesAreSplitAndTrimmed() {
    assertThat(VehicleErrors.fatalErrorTypes(withFatalErrors("eStop, adapterLostNavigation")))
        .as("StateMappings.toErrorPropertyValue joins with ', ', so the separator carries a space"
            + " that must not become part of the error type")
        .containsExactlyInAnyOrder("eStop", "adapterLostNavigation");
  }

  @Test
  void lostNavigationOnItsOwnIsTheOnlyFatalError() {
    assertThat(VehicleErrors.isOnlyLostNavigation(withFatalErrors("adapterLostNavigation")))
        .isTrue();
  }

  @Test
  void lostNavigationAlongsideAnotherFatalErrorIsNotTheOnlyOne() {
    assertThat(VehicleErrors.isOnlyLostNavigation(withFatalErrors("eStop, adapterLostNavigation")))
        .as("a vehicle that is also emergency-stopped cannot drive, so it must not be treated as"
            + " recoverable: its order stays, and the fleet plan walls it off")
        .isFalse();
  }

  @Test
  void vehicleWithoutErrorsHasNoLostNavigationAtAll() {
    assertThat(VehicleErrors.isOnlyLostNavigation(new Vehicle("V01"))).isFalse();
  }
}
