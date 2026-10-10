package org.opentcs.fms.kernel.rerouting;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.MovementCommand;
import org.opentcs.drivers.vehicle.VehicleController;
import org.opentcs.drivers.vehicle.VehicleControllerPool;

import jakarta.inject.Inject;

public class CommittedRouteFrontier {

  private final VehicleControllerPool vehicleControllerPool;

  @Inject
  public CommittedRouteFrontier(VehicleControllerPool vehicleControllerPool) {
    this.vehicleControllerPool = vehicleControllerPool;
  }

  public int frozenStepIndex(Vehicle vehicle, int fallbackIndex) {
    try {
      VehicleController controller = vehicleControllerPool.getVehicleController(vehicle.getName());
      if (controller == null) {
        return fallbackIndex;
      }
      Optional<MovementCommand> pending = controller.getInteractionsPendingCommand();
      if (pending.isPresent()) {
        return Math.max(fallbackIndex, pending.get().getStep().getRouteIndex());
      }
      List<MovementCommand> sent = new ArrayList<>(controller.getCommandsSent());
      if (!sent.isEmpty()) {
        return Math.max(fallbackIndex, sent.get(sent.size() - 1).getStep().getRouteIndex());
      }
    } catch (RuntimeException e) {
      return fallbackIndex;
    }
    return fallbackIndex;
  }
}
