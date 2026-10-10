package org.opentcs.fms.kernel;

import static java.util.Objects.requireNonNull;

import java.util.Objects;

import jakarta.inject.Inject;

import org.opentcs.components.kernel.KernelExtension;
import org.opentcs.customizations.ApplicationEventBus;
import org.opentcs.data.TCSObjectEvent;
import org.opentcs.data.model.Vehicle;
import org.opentcs.util.event.EventBus;
import org.opentcs.util.event.EventHandler;

public class VehicleMoveListener
    implements
      KernelExtension,
      EventHandler {

  private static final org.slf4j.Logger logger =
      org.slf4j.LoggerFactory.getLogger(VehicleMoveListener.class);

  private static final boolean ON = Boolean.parseBoolean(
      System.getProperty("fms.log.vehicleMoves", "true"));

  private static final String UNLOCALISED = "--";

  private final EventBus eventBus;
  private boolean initialized;

  @Inject
  public VehicleMoveListener(
      @ApplicationEventBus
      EventBus eventBus) {
    this.eventBus = requireNonNull(eventBus, "eventBus");
  }

  @Override
  public void initialize() {
    if (initialized) {
      return;
    }
    initialized = true;
    eventBus.subscribe(this);
    logger.info(
        "[FMS move] timestamping every cell a vehicle enters, so a rotation can be placed in"
            + " wall-clock time instead of in plan ticks");
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
    if (!ON
        || !(eventObject instanceof TCSObjectEvent event)
        || event.getType() != TCSObjectEvent.Type.OBJECT_MODIFIED
        || !(event.getCurrentObjectState() instanceof Vehicle current)
        || !(event.getPreviousObjectState() instanceof Vehicle previous)) {
      return;
    }

    String left = positionOf(previous);
    String entered = positionOf(current);
    if (Objects.equals(left, entered)) {
      return;
    }

    logger.info(
        "[FMS move] {} {}->{} loaded={} angle={}",
        current.getName(), left, entered, carriesCargo(current),
        Math.round(current.getPose().getOrientationAngle()));
  }

  private static String positionOf(Vehicle vehicle) {
    return vehicle.getCurrentPosition() == null
        ? UNLOCALISED
        : vehicle.getCurrentPosition().getName();
  }

  private static boolean carriesCargo(Vehicle vehicle) {
    return vehicle.getLoadHandlingDevices().stream().anyMatch(device -> device.isFull());
  }
}
