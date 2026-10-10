package org.opentcs.fms.kernel.transformers;

import static java.util.Objects.requireNonNull;

import jakarta.annotation.Nonnull;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.IncomingPoseTransformer;
import org.opentcs.drivers.vehicle.MovementCommandTransformer;
import org.opentcs.drivers.vehicle.VehicleDataTransformerFactory;

public class LaserActionTransformerFactory
    implements
    VehicleDataTransformerFactory {

  static final String FACTORY_NAME = "FMS_LASER_TRANSFORMER";
  private static final org.slf4j.Logger logger = org.slf4j.LoggerFactory.getLogger(LaserActionTransformerFactory.class);

  public LaserActionTransformerFactory() {
  }

  @Override
  @Nonnull
  public String getName() {
    return FACTORY_NAME;
  }

  @Override
  @Nonnull
  public MovementCommandTransformer createMovementCommandTransformer(
      @Nonnull Vehicle vehicle) {
    requireNonNull(vehicle, "vehicle");
    return new LaserActionTransformer();
  }

  @Override
  @Nonnull
  public IncomingPoseTransformer createIncomingPoseTransformer(
      @Nonnull Vehicle vehicle) {
    requireNonNull(vehicle, "vehicle");

    return pose -> pose;
  }

  @Override
  public boolean providesTransformersFor(
      @Nonnull Vehicle vehicle) {
    requireNonNull(vehicle, "vehicle");

    return true;
  }
}
