package org.opentcs.fms.kernel;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.Mockito.mock;
import static org.opentcs.commadapter.vehicle.vda5050.v2_0.ObjectProperties.PROPKEY_VEHICLE_ERRORS_FATAL;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.components.kernel.services.TCSObjectService;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Vehicle;
import org.opentcs.fms.kernel.vda5050.VehicleErrors;
import org.opentcs.strategies.basic.dispatching.DefaultDispatcherConfiguration;
import org.opentcs.strategies.basic.dispatching.OrderReservationPool;

class FmsIsAvailableForAnyOrderTest {

  private FmsIsAvailableForAnyOrder filter;

  @BeforeEach
  void setUp() {
    filter = new FmsIsAvailableForAnyOrder(
        mock(TCSObjectService.class),
        mock(OrderReservationPool.class),
        mock(DefaultDispatcherConfiguration.class));
  }

  private static Vehicle localisedVehicle() {
    return new Vehicle("V01")
        .withIntegrationLevel(Vehicle.IntegrationLevel.TO_BE_UTILIZED)
        .withCurrentPosition(new Point("0014").getReference())
        .withState(Vehicle.State.IDLE);
  }

  private static Vehicle lostNavigationVehicle() {
    return localisedVehicle()
        .withState(Vehicle.State.ERROR)
        .withProperty(PROPKEY_VEHICLE_ERRORS_FATAL, VehicleErrors.LOST_NAVIGATION);
  }

  @Test
  void anIdleVehicleStaysAvailableJustAsOpenTcsDecides() {
    assertThat(filter.test(localisedVehicle())).isTrue();
  }

  @Test
  void aVehicleThatOnlyLostNavigationIsOfferedOrdersDespiteBeingInError() {
    assertThat(filter.test(lostNavigationVehicle()))
        .as("it stands on a known point and accepting an order is what clears the error, so"
            + " excluding it would strand both the vehicle and the goods it was carrying")
        .isTrue();
  }

  @Test
  void aVehicleWithADifferentFatalErrorStaysUnavailable() {
    assertThat(filter.test(localisedVehicle()
        .withState(Vehicle.State.ERROR)
        .withProperty(PROPKEY_VEHICLE_ERRORS_FATAL, "eStop")))
        .isFalse();
  }

  @Test
  void lostNavigationAlongsideAnotherFatalErrorStaysUnavailable() {
    assertThat(filter.test(localisedVehicle()
        .withState(Vehicle.State.ERROR)
        .withProperty(
            PROPKEY_VEHICLE_ERRORS_FATAL, "eStop, " + VehicleErrors.LOST_NAVIGATION)))
        .as("an emergency-stopped vehicle cannot drive whatever else is true of it")
        .isFalse();
  }

  @Test
  void aVehicleInErrorWithoutTheErrorPropertyStaysUnavailable() {
    assertThat(filter.test(localisedVehicle().withState(Vehicle.State.ERROR))).isFalse();
  }

  @Test
  void lostNavigationExcusesTheErrorStateAndNothingElse() {
    assertThat(filter.test(lostNavigationVehicle().withPaused(true)))
        .as("every condition openTCS checks other than the vehicle state must still apply")
        .isFalse();
    assertThat(filter.test(lostNavigationVehicle().withCurrentPosition(null)))
        .isFalse();
    assertThat(filter.test(lostNavigationVehicle()
        .withIntegrationLevel(Vehicle.IntegrationLevel.TO_BE_RESPECTED)))
        .isFalse();
  }
}
