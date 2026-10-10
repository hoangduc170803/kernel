package org.opentcs.fms.kernel.rerouting;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.Route;
import org.opentcs.drivers.vehicle.MovementCommand;
import org.opentcs.drivers.vehicle.VehicleController;
import org.opentcs.drivers.vehicle.VehicleControllerPool;

class CommittedRouteFrontierTest {

  @Test
  void pendingInteractionCommandTakesPrecedenceOverSentCommands() {
    Vehicle vehicle = new Vehicle("Vehicle");
    VehicleControllerPool pool = mock(VehicleControllerPool.class);
    VehicleController controller = mock(VehicleController.class);
    MovementCommand pending = commandAt(2);
    MovementCommand lastSent = commandAt(4);
    when(controller.getInteractionsPendingCommand()).thenReturn(Optional.of(pending));
    when(controller.getCommandsSent()).thenReturn(new LinkedList<>(List.of(lastSent)));
    when(pool.getVehicleController(vehicle.getName())).thenReturn(controller);

    assertThat(new CommittedRouteFrontier(pool).frozenStepIndex(vehicle, -1)).isEqualTo(2);
  }

  @Test
  void usesTheFallbackWhenThereIsNoController() {
    Vehicle vehicle = new Vehicle("Vehicle");
    VehicleControllerPool pool = mock(VehicleControllerPool.class);

    assertThat(new CommittedRouteFrontier(pool).frozenStepIndex(vehicle, 3)).isEqualTo(3);
  }

  private static MovementCommand commandAt(int routeIndex) {
    MovementCommand command = mock(MovementCommand.class);
    when(command.getStep()).thenReturn(new Route.Step(
        null, new Point("source"), new Point("destination"), Vehicle.Orientation.FORWARD,
        routeIndex, 1));
    return command;
  }
}
