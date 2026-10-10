package org.opentcs.fms.kernel.transformers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assumptions.assumeTrue;
import static org.opentcs.fms.kernel.transformers.LaserActionTransformer.LASER_ACTION_PROPERTY;
import static org.opentcs.fms.kernel.transformers.LaserActionTransformer.LASER_OFF_ACTION;
import static org.opentcs.fms.kernel.transformers.LaserActionTransformer.LASER_ON_ACTION;
import static org.opentcs.fms.kernel.transformers.LaserActionTransformer.POINT_LASER_PROPERTY;

import java.io.File;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.data.model.Path;
import org.opentcs.data.model.Point;
import org.opentcs.data.order.DriveOrder;
import org.opentcs.data.order.Route;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.drivers.vehicle.MovementCommand;
import org.opentcs.fms.routing.RoadmapXmlLoader;

class LaserActionTransformerRealMapTest {

  private static final String MAP_FILE = "v7-vda5050-test.xml";
  private static final String MOVE = MovementCommand.MOVE_OPERATION;
  private static final String START_CHARGING = "startCharging";

  private static PlantModelXmlFixture map;

  private LaserActionTransformer transformer;

  @BeforeAll
  static void loadMap()
      throws Exception {
    File file = RoadmapXmlLoader.findMap(MAP_FILE);
    assumeTrue(file != null, MAP_FILE + " not found; skipping");
    map = PlantModelXmlFixture.load(file);
  }

  @BeforeEach
  void setUp() {
    transformer = new LaserActionTransformer();
  }

  @Test
  void shouldKeepLaserOnAlongAStraightRunOfTheMainline() {
    Route route = map.routeThrough("0004", "0005", "0006", "0007");

    for (int routeIndex : List.of(0, 1)) {
      Route.Step step = transformedStep(route, routeIndex, MOVE);

      assertEquals(LASER_ON_ACTION, laserAction(step.getDestinationPoint()));
      assertEquals(LASER_ON_ACTION, laserAction(step.getPath()));
    }
  }

  @Test
  void shouldTurnLaserOffOnTheStepEnteringARealTurningPoint() {
    Route route = map.routeThrough("0004", "0005", "3009");

    Route.Step stepIntoTheTurn = transformedStep(route, 0, MOVE);

    assertEquals(LASER_OFF_ACTION, laserAction(stepIntoTheTurn.getDestinationPoint()));
    assertEquals(LASER_OFF_ACTION, laserAction(stepIntoTheTurn.getPath()));
  }

  @Test
  void shouldTurnLaserBackOnOnceTheTurnIsBehindTheVehicle() {
    Route route = map.routeThrough("0004", "0005", "3009", "3010");

    Route.Step stepIntoTheTurn = transformedStep(route, 0, MOVE);
    Route.Step stepLeavingTheTurn = transformedStep(route, 1, MOVE);
    Route.Step stepAfterwards = transformedStep(route, 2, MOVE);

    assertEquals(LASER_OFF_ACTION, laserAction(stepIntoTheTurn.getDestinationPoint()));
    assertEquals(LASER_ON_ACTION, laserAction(stepLeavingTheTurn.getDestinationPoint()));
    assertEquals(LASER_ON_ACTION, laserAction(stepLeavingTheTurn.getPath()));
    assertEquals(LASER_ON_ACTION, laserAction(stepAfterwards.getDestinationPoint()));
  }

  @Test
  void shouldTurnLaserOffAtTheSourcePointOfAFirstStepThatEntersATurn() {
    Route route = map.routeThrough("0004", "0005", "3009");

    Route.Step step = transformedStep(route, 0, MOVE);

    assertEquals(LASER_OFF_ACTION, laserAction(step.getSourcePoint()));
  }

  @Test
  void shouldTurnLaserOffWhenParkingOnARealParkPosition() {
    assertEquals(Point.Type.PARK_POSITION, map.point("0047").getType());
    Route route = map.routeThrough("0045", "0046", "0047");

    Route.Step stepIntoThePark = transformedStep(route, 1, MOVE);

    assertEquals(LASER_OFF_ACTION, laserAction(stepIntoThePark.getDestinationPoint()));
    assertEquals(LASER_OFF_ACTION, laserAction(stepIntoThePark.getPath()));
  }

  @Test
  void shouldKeepLaserOnWhileStillApproachingTheParkPosition() {
    Route route = map.routeThrough("0045", "0046", "0047");

    Route.Step approachStep = transformedStep(route, 0, MOVE);

    assertEquals(LASER_ON_ACTION, laserAction(approachStep.getDestinationPoint()));
    assertEquals(LASER_ON_ACTION, laserAction(approachStep.getPath()));
  }

  @Test
  void shouldTurnLaserOffWhenReachingARealChargingPoint() {
    for (String chargingPoint : List.of("0200", "0210", "0220", "0230")) {
      assertEquals("off", map.point(chargingPoint).getProperty(POINT_LASER_PROPERTY));
      assertEquals(Point.Type.HALT_POSITION, map.point(chargingPoint).getType());
    }

    Route route = map.routeThrough("0046", "0201", "0200");

    Route.Step stepIntoTheCharger = transformedStep(route, 1, START_CHARGING);

    assertEquals(LASER_OFF_ACTION, laserAction(stepIntoTheCharger.getDestinationPoint()));
    assertEquals(LASER_OFF_ACTION, laserAction(stepIntoTheCharger.getPath()));
  }

  @Test
  void shouldNotTreatBackingIntoTheChargerAsATurn() {
    Route route = map.routeThrough("0201", "0200");

    Route.Step onlyStep = transformedStep(route, 0, START_CHARGING);

    assertEquals(LASER_OFF_ACTION, laserAction(onlyStep.getDestinationPoint()));
    assertEquals(LASER_ON_ACTION, laserAction(onlyStep.getSourcePoint()));
  }

  @Test
  void shouldKeepLaserOnAtAPointTheMapDoesNotFlag() {
    assertEquals(null, map.point("0005").getProperty(POINT_LASER_PROPERTY));

    Route.Step step = transformedStep(map.routeThrough("0004", "0005", "0006"), 0, MOVE);

    assertEquals(LASER_ON_ACTION, laserAction(step.getDestinationPoint()));
  }

  private Route.Step transformedStep(Route route, int routeIndex, String finalOperation) {
    return transformer.apply(command(route, routeIndex, finalOperation)).getStep();
  }

  private static String laserAction(Point point) {
    return point.getProperty(LASER_ACTION_PROPERTY);
  }

  private static String laserAction(Path path) {
    return path.getProperty(LASER_ACTION_PROPERTY);
  }

  private static MovementCommand command(Route route, int routeIndex, String finalOperation) {
    Route.Step step = route.getSteps().get(routeIndex);
    Point finalDestination = route.getFinalDestinationPoint();

    return new MovementCommand(
        new TransportOrder("transportOrder", List.of()),
        new DriveOrder("order", new DriveOrder.Destination(finalDestination.getReference()))
            .withRoute(route),
        step,
        MOVE,
        null,
        routeIndex == route.getSteps().size() - 1,
        null,
        finalDestination,
        finalOperation,
        Map.of());
  }
}
