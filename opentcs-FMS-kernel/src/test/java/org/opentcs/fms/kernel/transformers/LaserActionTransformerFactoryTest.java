package org.opentcs.fms.kernel.transformers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.data.model.Pose;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;

class LaserActionTransformerFactoryTest {

  private LaserActionTransformerFactory factory;
  private Vehicle vehicle;

  @BeforeEach
  void setUp() {
    factory = new LaserActionTransformerFactory();
    vehicle = new Vehicle("Vehicle-0001");
  }

  @Test
  void shouldBeSelectableByTheNameExpectedInThePlantModel() {
    assertEquals("FMS_LASER_TRANSFORMER", factory.getName());
  }

  @Test
  void shouldProvideTransformersForEveryVehicle() {
    assertTrue(factory.providesTransformersFor(vehicle));
  }

  @Test
  void shouldCreateALaserActionMovementCommandTransformer() {
    assertInstanceOf(
        LaserActionTransformer.class, factory.createMovementCommandTransformer(vehicle));
  }

  @Test
  void shouldLeaveIncomingPosesUntouched() {
    Pose pose = new Pose(new Triple(1, 2, 3), 45.0);

    assertSame(pose, factory.createIncomingPoseTransformer(vehicle).apply(pose));
  }
}
