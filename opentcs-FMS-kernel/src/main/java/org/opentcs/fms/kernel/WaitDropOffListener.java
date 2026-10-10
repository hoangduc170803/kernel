package org.opentcs.fms.kernel;

import static java.util.Objects.requireNonNull;

import jakarta.inject.Inject;

import java.util.Objects;
import java.util.concurrent.ScheduledExecutorService;

import org.opentcs.components.kernel.KernelExtension;
import org.opentcs.components.kernel.services.InternalTCSObjectService;
import org.opentcs.customizations.ApplicationEventBus;
import org.opentcs.customizations.kernel.KernelExecutor;
import org.opentcs.data.TCSObjectEvent;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.util.event.EventBus;
import org.opentcs.util.event.EventHandler;

public class WaitDropOffListener
    implements
      KernelExtension,
      EventHandler {

  public static final String WAIT_DROPOFF_PROPERTY = "fms:waitDropOff";
  private static final String APPROACH_ORDER_PREFIX = "APPROACH-";
  private static final String WAITING = "true";
  private static final String NOT_WAITING = "false";

  private static final org.slf4j.Logger logger =
      org.slf4j.LoggerFactory.getLogger(WaitDropOffListener.class);

  private final EventBus eventBus;
  private final InternalTCSObjectService objectService;
  private final ScheduledExecutorService kernelExecutor;
  private boolean initialized;

  @Inject
  public WaitDropOffListener(
      @ApplicationEventBus
      EventBus eventBus,
      InternalTCSObjectService objectService,
      @KernelExecutor
      ScheduledExecutorService kernelExecutor
  ) {
    this.eventBus = requireNonNull(eventBus, "eventBus");
    this.objectService = requireNonNull(objectService, "objectService");
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
        "[FMS flag] watching order assignments to maintain {} on vehicles", WAIT_DROPOFF_PROPERTY);
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
    if (!(eventObject instanceof TCSObjectEvent event)) {
      return;
    }
    if (!(event.getCurrentObjectState() instanceof TransportOrder order)) {
      return;
    }
    if (!justBecameProcessedByVehicle(event, order)) {
      return;
    }

    String vehicleName = order.getProcessingVehicle().getName();
    String orderName = order.getName();
    kernelExecutor.submit(() -> applyFlag(vehicleName, orderName));
  }

  private static boolean justBecameProcessedByVehicle(TCSObjectEvent event, TransportOrder order) {
    return event.getType() == TCSObjectEvent.Type.OBJECT_MODIFIED
        && isProcessedByVehicle(order)
        && event.getPreviousObjectState() instanceof TransportOrder previous
        && !isProcessedByVehicle(previous);
  }

  private static boolean isProcessedByVehicle(TransportOrder order) {
    return order.hasState(TransportOrder.State.BEING_PROCESSED)
        && order.getProcessingVehicle() != null;
  }

  private void applyFlag(String vehicleName, String orderName) {
    objectService.fetch(Vehicle.class, vehicleName).ifPresent(vehicle -> {
      String value = orderName.startsWith(APPROACH_ORDER_PREFIX) ? WAITING : NOT_WAITING;
      if (Objects.equals(vehicle.getProperty(WAIT_DROPOFF_PROPERTY), value)) {
        return;
      }
      objectService.updateObjectProperty(vehicle.getReference(), WAIT_DROPOFF_PROPERTY, value);
      logger.info(
          "[FMS flag] {} {}={} (order {} started)",
          vehicleName, WAIT_DROPOFF_PROPERTY, value, orderName);
    });
  }
}
