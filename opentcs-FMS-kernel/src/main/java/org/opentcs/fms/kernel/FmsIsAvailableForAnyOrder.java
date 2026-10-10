package org.opentcs.fms.kernel;

import org.opentcs.components.kernel.services.TCSObjectService;
import org.opentcs.data.model.Vehicle;
import org.opentcs.fms.kernel.vda5050.VehicleErrors;
import org.opentcs.strategies.basic.dispatching.DefaultDispatcherConfiguration;
import org.opentcs.strategies.basic.dispatching.OrderReservationPool;
import org.opentcs.strategies.basic.dispatching.selection.vehicles.IsAvailableForAnyOrder;

import jakarta.inject.Inject;

public class FmsIsAvailableForAnyOrder
    extends
      IsAvailableForAnyOrder {

  private static final org.slf4j.Logger logger =
      org.slf4j.LoggerFactory.getLogger(FmsIsAvailableForAnyOrder.class);

  @Inject
  public FmsIsAvailableForAnyOrder(
      TCSObjectService objectService,
      OrderReservationPool orderReservationPool,
      DefaultDispatcherConfiguration configuration) {
    super(objectService, orderReservationPool, configuration);
  }

  @Override
  public boolean test(Vehicle vehicle) {
    if (super.test(vehicle)) {
      return true;
    }
    if (!reportsOnlyLostNavigation(vehicle)) {
      return false;
    }
    boolean availableApartFromLostNavigation = super.test(vehicle.withState(Vehicle.State.IDLE));
    if (availableApartFromLostNavigation) {
      logger.info(
          "[FMS lostnav] {} is offered orders despite reporting {}: it can no longer follow the"
              + " route it was given, but it knows where it stands and accepting a new order is"
              + " what clears the error",
          vehicle.getName(), VehicleErrors.LOST_NAVIGATION);
    }
    return availableApartFromLostNavigation;
  }

  private static boolean reportsOnlyLostNavigation(Vehicle vehicle) {
    return vehicle.hasState(Vehicle.State.ERROR) && VehicleErrors.isOnlyLostNavigation(vehicle);
  }
}
