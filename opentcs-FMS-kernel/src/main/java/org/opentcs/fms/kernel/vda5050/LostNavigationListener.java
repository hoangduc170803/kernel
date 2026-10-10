package org.opentcs.fms.kernel.vda5050;

import static java.util.Objects.requireNonNull;

import java.util.concurrent.ScheduledExecutorService;

import jakarta.inject.Inject;

import org.opentcs.components.kernel.KernelExtension;
import org.opentcs.components.kernel.services.DispatcherService;
import org.opentcs.components.kernel.services.InternalTCSObjectService;
import org.opentcs.customizations.ApplicationEventBus;
import org.opentcs.customizations.kernel.KernelExecutor;
import org.opentcs.data.TCSObjectEvent;
import org.opentcs.data.model.Vehicle;
import org.opentcs.util.event.EventBus;
import org.opentcs.util.event.EventHandler;

public class LostNavigationListener
    implements
      KernelExtension,
      EventHandler {

  private static final org.slf4j.Logger logger =
      org.slf4j.LoggerFactory.getLogger(LostNavigationListener.class);

  private final EventBus eventBus;
  private final InternalTCSObjectService objectService;
  private final DispatcherService dispatcherService;
  private final ScheduledExecutorService kernelExecutor;
  private boolean initialized;

  @Inject
  public LostNavigationListener(
      @ApplicationEventBus
      EventBus eventBus,
      InternalTCSObjectService objectService,
      DispatcherService dispatcherService,
      @KernelExecutor
      ScheduledExecutorService kernelExecutor) {
    this.eventBus = requireNonNull(eventBus, "eventBus");
    this.objectService = requireNonNull(objectService, "objectService");
    this.dispatcherService = requireNonNull(dispatcherService, "dispatcherService");
    this.kernelExecutor = requireNonNull(kernelExecutor, "kernelExecutor");
  }

  @Override
  public void initialize() {
    if (initialized) {
      return;
    }
    initialized = true;
    eventBus.subscribe(this);
    logger.info(
        "[FMS lostnav] watching vehicles to withdraw the order of any that reports {}",
        VehicleErrors.LOST_NAVIGATION);
  }

  @Override
  public boolean isInitialized() {
    return initialized;
  }

  @Override
  public void terminate() {
    if (!initialized) {
      return;
    }
    initialized = false;
    eventBus.unsubscribe(this);
  }

  @Override
  public void onEvent(Object eventObject) {
    if (!(eventObject instanceof TCSObjectEvent event) || !justLostNavigation(event)) {
      return;
    }

    String vehicleName = ((Vehicle) event.getCurrentObjectState()).getName();
    kernelExecutor.submit(() -> withdrawOrderOf(vehicleName));
  }

  private static boolean justLostNavigation(TCSObjectEvent event) {
    return event.getType() == TCSObjectEvent.Type.OBJECT_MODIFIED
        && event.getCurrentObjectState() instanceof Vehicle current
        && VehicleErrors.isOnlyLostNavigation(current)
        && event.getPreviousObjectState() instanceof Vehicle previous
        && !VehicleErrors.isOnlyLostNavigation(previous);
  }

  private void withdrawOrderOf(String vehicleName) {
    objectService.fetch(Vehicle.class, vehicleName).ifPresent(vehicle -> {
      if (!VehicleErrors.isOnlyLostNavigation(vehicle) || !vehicle.isProcessingOrder()) {
        logger.debug(
            "[FMS lostnav] {} no longer needs its order withdrawn (lostNavigation={},"
                + " processingOrder={})",
            vehicleName, VehicleErrors.isOnlyLostNavigation(vehicle),
            vehicle.isProcessingOrder());
        return;
      }
      logger.warn(
          "[FMS lostnav] {} reports {} at {} while processing {}: withdrawing that order so the"
              + " commands it can no longer execute are dropped and WES can decide what happens"
              + " to the goods",
          vehicleName, VehicleErrors.LOST_NAVIGATION,
          vehicle.getCurrentPosition() == null ? "an unknown point"
              : vehicle.getCurrentPosition().getName(),
          vehicle.getTransportOrder().getName());
      dispatcherService.withdrawByVehicle(vehicle.getReference(), true);
    });
  }
}
