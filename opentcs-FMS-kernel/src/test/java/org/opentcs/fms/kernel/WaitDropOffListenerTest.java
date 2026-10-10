package org.opentcs.fms.kernel;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.ScheduledExecutorService;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.components.kernel.services.InternalTCSObjectService;
import org.opentcs.data.TCSObjectEvent;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.util.event.EventBus;

class WaitDropOffListenerTest {

  private static final String APPROACH_ORDER = "APPROACH-0042";
  private static final String DROPOFF_ORDER = "DROPOFF-0042";
  private static final String VEHICLE_NAME = "Vehicle-0006";
  private static final String WAIT_DROPOFF = WaitDropOffListener.WAIT_DROPOFF_PROPERTY;
  private static final String WAITING = "true";
  private static final String NOT_WAITING = "false";

  private EventBus eventBus;
  private InternalTCSObjectService objectService;
  private ScheduledExecutorService kernelExecutor;
  private WaitDropOffListener listener;
  private Vehicle vehicle;

  @BeforeEach
  void setUp() {
    eventBus = mock(EventBus.class);
    objectService = mock(InternalTCSObjectService.class);
    kernelExecutor = mock(ScheduledExecutorService.class);
    doAnswer(call -> {
      call.<Runnable>getArgument(0).run();
      return null;
    }).when(kernelExecutor).submit(any(Runnable.class));
    registerVehicle(new Vehicle(VEHICLE_NAME));
    listener = new WaitDropOffListener(eventBus, objectService, kernelExecutor);
  }

  @Test
  void writesWaitFlagWhenTheVehicleIsAssignedAfterTheOrderStartedBeingProcessed() {
    TransportOrder unassigned = beingProcessedWithoutVehicle(APPROACH_ORDER);
    TransportOrder assigned = beingProcessedByTheVehicle(APPROACH_ORDER);

    listener.onEvent(modification(unassigned, assigned));

    assertPropertyWritten(WAITING);
  }

  @Test
  void writesNothingWhenTheOrderStartsBeingProcessedWithoutAVehicleYet() {
    TransportOrder dispatchable = dispatchable(APPROACH_ORDER);
    TransportOrder unassigned = beingProcessedWithoutVehicle(APPROACH_ORDER);

    listener.onEvent(modification(dispatchable, unassigned));

    assertNoPropertyWritten();
  }

  @Test
  void writesWaitFlagWhenTheStateAndTheVehicleArriveInTheSameEvent() {
    TransportOrder dispatchable = dispatchable(APPROACH_ORDER);
    TransportOrder assigned = beingProcessedByTheVehicle(APPROACH_ORDER);

    listener.onEvent(modification(dispatchable, assigned));

    assertPropertyWritten(WAITING);
  }

  @Test
  void writesNotWaitingForOrdersOutsideTheApproachPrefix() {
    TransportOrder unassigned = beingProcessedWithoutVehicle(DROPOFF_ORDER);
    TransportOrder assigned = beingProcessedByTheVehicle(DROPOFF_ORDER);

    listener.onEvent(modification(unassigned, assigned));

    assertPropertyWritten(NOT_WAITING);
  }

  @Test
  void writesNothingWhenTheOrderWasAlreadyBeingProcessedByThatVehicle() {
    TransportOrder assigned = beingProcessedByTheVehicle(APPROACH_ORDER);
    TransportOrder stillAssigned = beingProcessedByTheVehicle(APPROACH_ORDER);

    listener.onEvent(modification(assigned, stillAssigned));

    assertNoPropertyWritten();
  }

  @Test
  void writesNothingWhenTheVehicleAlreadyCarriesTheSameValue() {
    registerVehicle(new Vehicle(VEHICLE_NAME).withProperty(WAIT_DROPOFF, WAITING));
    TransportOrder unassigned = beingProcessedWithoutVehicle(APPROACH_ORDER);
    TransportOrder assigned = beingProcessedByTheVehicle(APPROACH_ORDER);

    listener.onEvent(modification(unassigned, assigned));

    assertNoPropertyWritten();
  }

  @Test
  void subscribesToTheEventBusOnInitialize() {
    listener.initialize();

    verify(eventBus).subscribe(listener);
  }

  private void registerVehicle(Vehicle registered) {
    vehicle = registered;
    when(objectService.fetch(Vehicle.class, VEHICLE_NAME)).thenReturn(Optional.of(registered));
  }

  private void assertPropertyWritten(String value) {
    verify(objectService).updateObjectProperty(vehicle.getReference(), WAIT_DROPOFF, value);
  }

  private void assertNoPropertyWritten() {
    verify(objectService, never()).updateObjectProperty(any(), anyString(), any());
  }

  private TransportOrder beingProcessedByTheVehicle(String orderName) {
    return beingProcessedWithoutVehicle(orderName).withProcessingVehicle(vehicle.getReference());
  }

  private static TransportOrder beingProcessedWithoutVehicle(String orderName) {
    return new TransportOrder(orderName, List.of())
        .withState(TransportOrder.State.BEING_PROCESSED);
  }

  private static TransportOrder dispatchable(String orderName) {
    return new TransportOrder(orderName, List.of())
        .withState(TransportOrder.State.DISPATCHABLE);
  }

  private static TCSObjectEvent modification(TransportOrder previous, TransportOrder current) {
    return new TCSObjectEvent(current, previous, TCSObjectEvent.Type.OBJECT_MODIFIED);
  }
}
