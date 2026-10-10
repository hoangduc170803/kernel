package org.opentcs.fms.kernel.transformers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.opentcs.fms.kernel.transformers.LaserActionTransformer.LASER_ACTION_BLOCKING_TYPE_PROPERTY;
import static org.opentcs.fms.kernel.transformers.LaserActionTransformer.LASER_ACTION_PROPERTY;
import static org.opentcs.fms.kernel.transformers.LaserActionTransformer.APPROACH_LEG;
import static org.opentcs.fms.kernel.transformers.LaserActionTransformer.LASER_OFF_ACTION;
import static org.opentcs.fms.kernel.transformers.LaserActionTransformer.LASER_ON_ACTION;
import static org.opentcs.fms.kernel.transformers.LaserActionTransformer.LEG_PROPERTY;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.commadapter.vehicle.vda5050.v2_0.message.common.BlockingType;
import org.opentcs.commadapter.vehicle.vda5050.v2_0.ordermapping.ActionTrigger;
import org.opentcs.commadapter.vehicle.vda5050.v2_0.ordermapping.ActionsMapping;
import org.opentcs.commadapter.vehicle.vda5050.v2_0.ordermapping.PropertyAction;
import org.opentcs.data.model.Path;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Pose;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.DriveOrder;
import org.opentcs.data.order.Route;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.drivers.vehicle.MovementCommand;

class LaserActionTransformerTest {

  private static final String MOVE = MovementCommand.MOVE_OPERATION;
  private static final String LIFT_UP = "liftUp";
  private static final String LIFT_DOWN = "liftDown";
  private static final String LEAVE_GOODS = "leaveGoods";
  private static final String BLOCKING_TYPE_NONE = "NONE";
  private static final String POINT_LASER_PROPERTY = "laser";
  private static final String NOP = MovementCommand.NO_OPERATION;

  private LaserActionTransformer transformer;

  @BeforeEach
  void setUp() {
    transformer = new LaserActionTransformer();
  }

  @Test
  void shouldTurnLaserOnAtSourcePointOfFirstStep() {
    Route.Step step = transformedStep(sixHaltPoints(), 0, MOVE);

    assertEquals(LASER_ON_ACTION, laserAction(step.getSourcePoint()));
    assertEquals(
        BLOCKING_TYPE_NONE,
        step.getSourcePoint().getProperty(LASER_ACTION_BLOCKING_TYPE_PROPERTY));
  }

  @Test
  void shouldTurnLaserOnForPlainMovement() {
    Route.Step step = transformedStep(sixHaltPoints(), 1, MOVE);

    assertEquals(LASER_ON_ACTION, laserAction(step.getDestinationPoint()));
    assertEquals(LASER_ON_ACTION, laserAction(step.getPath()));
    assertEquals(
        BLOCKING_TYPE_NONE,
        step.getDestinationPoint().getProperty(LASER_ACTION_BLOCKING_TYPE_PROPERTY));
    assertEquals(
        BLOCKING_TYPE_NONE,
        step.getPath().getProperty(LASER_ACTION_BLOCKING_TYPE_PROPERTY));
  }

  @Test
  void shouldNotTouchSourcePointOfLaterSteps() {
    Route.Step step = transformedStep(sixHaltPoints(), 1, MOVE);

    assertNull(laserAction(step.getSourcePoint()));
  }

  @Test
  void shouldTurnLaserOnForEveryStepOfAPlainMovementOrder() {
    for (int routeIndex : List.of(0, 1, 2, 3, 4)) {
      Route.Step step = transformedStep(sixHaltPoints(), routeIndex, MOVE);

      assertEquals(LASER_ON_ACTION, laserAction(step.getDestinationPoint()));
      assertEquals(LASER_ON_ACTION, laserAction(step.getPath()));
    }
  }

  @Test
  void shouldTurnLaserOffOnTheLastThreeStepsBeforeAHandlingOperation() {
    for (String finalOperation : List.of(LIFT_UP, LIFT_DOWN, LEAVE_GOODS)) {
      for (int routeIndex : List.of(2, 3, 4)) {
        Route.Step step = transformedStep(sixHaltPoints(), routeIndex, finalOperation);

        assertEquals(LASER_OFF_ACTION, laserAction(step.getDestinationPoint()));
        assertEquals(LASER_OFF_ACTION, laserAction(step.getPath()));
      }
    }
  }

  @Test
  void shouldKeepLaserOnBeforeTheHandlingEndSegment() {
    for (String finalOperation : List.of(LIFT_UP, LIFT_DOWN, LEAVE_GOODS)) {
      for (int routeIndex : List.of(0, 1)) {
        Route.Step step = transformedStep(sixHaltPoints(), routeIndex, finalOperation);

        assertEquals(LASER_ON_ACTION, laserAction(step.getDestinationPoint()));
        assertEquals(LASER_ON_ACTION, laserAction(step.getPath()));
      }
    }
  }

  @Test
  void shouldTurnLaserOffOnTheLastThreeStepsOfAnApproachLeg() {
    for (int routeIndex : List.of(2, 3, 4)) {
      Route.Step step = transformedApproachStep(sixHaltPoints(), routeIndex);

      assertEquals(LASER_OFF_ACTION, laserAction(step.getDestinationPoint()));
      assertEquals(LASER_OFF_ACTION, laserAction(step.getPath()));
    }
  }

  @Test
  void shouldKeepLaserOnEarlierInAnApproachLeg() {
    for (int routeIndex : List.of(0, 1)) {
      Route.Step step = transformedApproachStep(sixHaltPoints(), routeIndex);

      assertEquals(LASER_ON_ACTION, laserAction(step.getDestinationPoint()));
      assertEquals(LASER_ON_ACTION, laserAction(step.getPath()));
    }
  }

  @Test
  void shouldTurnLaserOnForALegThatIsNeitherApproachNorHandling() {
    Route.Step step = transformer
        .apply(command(routeOf(sixHaltPoints()), 4, NOP, Map.of(LEG_PROPERTY, "DROPOFF")))
        .getStep();

    assertEquals(LASER_ON_ACTION, laserAction(step.getDestinationPoint()));
    assertEquals(LASER_ON_ACTION, laserAction(step.getPath()));
  }

  @Test
  void shouldTurnLaserOffAtAPointTheMapFlagsAsLaserOff() {
    List<Point> points = sixHaltPoints();
    points.set(2, points.get(2).withProperty(POINT_LASER_PROPERTY, "off"));

    Route.Step step = transformedStep(points, 1, MOVE);

    assertEquals(LASER_OFF_ACTION, laserAction(step.getDestinationPoint()));
    assertEquals(LASER_OFF_ACTION, laserAction(step.getPath()));
  }

  @Test
  void shouldKeepLaserOnAtAPointTheMapFlagsAsLaserOn() {
    List<Point> points = sixHaltPoints();
    points.set(2, points.get(2).withProperty(POINT_LASER_PROPERTY, "on"));

    Route.Step step = transformedStep(points, 1, MOVE);

    assertEquals(LASER_ON_ACTION, laserAction(step.getDestinationPoint()));
  }

  @Test
  void shouldTurnLaserOffAtAParkPositionOnly() {
    List<Point> points = sixHaltPoints();
    points.set(2, points.get(2).withType(Point.Type.PARK_POSITION));

    Route.Step stepIntoThePark = transformedStep(points, 1, MOVE);
    Route.Step stepLeavingThePark = transformedStep(points, 2, MOVE);

    assertEquals(LASER_OFF_ACTION, laserAction(stepIntoThePark.getDestinationPoint()));
    assertEquals(LASER_OFF_ACTION, laserAction(stepIntoThePark.getPath()));
    assertEquals(LASER_ON_ACTION, laserAction(stepLeavingThePark.getDestinationPoint()));
  }

  @Test
  void shouldTurnLaserOffOnTheStepEnteringAQuarterTurn() {
    Route route = routeOf(pointsAt(coordinates(0, 0), coordinates(1000, 0), coordinates(1000, 1000)));

    Route.Step step = transformer.apply(command(route, 0, MOVE)).getStep();

    assertEquals(LASER_OFF_ACTION, laserAction(step.getDestinationPoint()));
    assertEquals(LASER_OFF_ACTION, laserAction(step.getPath()));
    assertEquals(LASER_OFF_ACTION, laserAction(step.getSourcePoint()));
  }

  @Test
  void shouldKeepLaserOnWhenTheRouteRunsStraightThroughAPoint() {
    Route route = routeOf(pointsAt(coordinates(0, 0), coordinates(1000, 0), coordinates(2000, 0)));

    Route.Step step = transformer.apply(command(route, 0, MOVE)).getStep();

    assertEquals(LASER_ON_ACTION, laserAction(step.getDestinationPoint()));
    assertEquals(LASER_ON_ACTION, laserAction(step.getPath()));
  }

  @Test
  void shouldNotTreatAReversalAsAQuarterTurn() {
    List<Point> points
        = pointsAt(coordinates(0, 0), coordinates(1000, 0), coordinates(2000, 0));
    Route route = new Route(
        List.of(
            stepBetween(points.get(0), points.get(1), Vehicle.Orientation.FORWARD, 0),
            stepBetween(points.get(1), points.get(2), Vehicle.Orientation.BACKWARD, 1)));

    Route.Step step = transformer.apply(command(route, 0, MOVE)).getStep();

    assertEquals(LASER_ON_ACTION, laserAction(step.getDestinationPoint()));
  }

  @Test
  void shouldTreatABackwardStepByTheDirectionOfTheVehicleNose() {
    List<Point> points
        = pointsAt(coordinates(0, 0), coordinates(1000, 0), coordinates(1000, 1000));
    Route route = new Route(
        List.of(
            stepBetween(points.get(0), points.get(1), Vehicle.Orientation.BACKWARD, 0),
            stepBetween(points.get(1), points.get(2), Vehicle.Orientation.FORWARD, 1)));

    Route.Step step = transformer.apply(command(route, 0, MOVE)).getStep();

    assertEquals(LASER_OFF_ACTION, laserAction(step.getDestinationPoint()));
  }

  @Test
  void shouldKeepLaserOnOnTheLastStepOfARouteThatHasNoSuccessor() {
    Route route = routeOf(pointsAt(coordinates(0, 0), coordinates(1000, 0), coordinates(1000, 1000)));

    Route.Step step = transformer.apply(command(route, 1, MOVE)).getStep();

    assertEquals(LASER_ON_ACTION, laserAction(step.getDestinationPoint()));
  }

  @Test
  void shouldApplyTheHandlingEndSegmentToAStationaryStep() {
    Route.Step plainStep = transformer.apply(stationaryCommand(MOVE)).getStep();
    Route.Step operationStep = transformer.apply(stationaryCommand(LIFT_DOWN)).getStep();

    assertEquals(LASER_ON_ACTION, laserAction(plainStep.getDestinationPoint()));
    assertEquals(LASER_OFF_ACTION, laserAction(operationStep.getDestinationPoint()));
    assertNull(plainStep.getPath());
    assertNull(plainStep.getSourcePoint());
  }

  @Test
  void shouldProduceTheSameResultForTheSameInput() {
    MovementCommand original = command(routeOf(sixHaltPoints()), 2, LIFT_UP);

    MovementCommand first = transformer.apply(original);
    MovementCommand second = transformer.apply(original);

    assertEquals(
        first.getStep().getDestinationPoint().getProperties(),
        second.getStep().getDestinationPoint().getProperties());
    assertEquals(
        first.getStep().getPath().getProperties(),
        second.getStep().getPath().getProperties());
    assertEquals(first, second);
    assertEquals(original, first);
  }

  @Test
  void shouldUseThePropertyKeysAndActionTypesTheDriverExpects() {
    assertEquals("vda5050:action.laser", LASER_ACTION_PROPERTY);
    assertEquals("vda5050:action.laser.blockingType", LASER_ACTION_BLOCKING_TYPE_PROPERTY);
    assertEquals("laserOn", LASER_ON_ACTION);
    assertEquals("laserOff", LASER_OFF_ACTION);
  }

  @Test
  void shouldMapToExactlyOneNonBlockingDriverActionPerNodeAndEdge() {
    Route.Step step = transformedStep(sixHaltPoints(), 2, LIFT_UP);

    assertSingleDriverAction(
        ActionsMapping.mapPropertyActions(step.getDestinationPoint()), LASER_OFF_ACTION);
    assertSingleDriverAction(ActionsMapping.mapPropertyActions(step.getPath()), LASER_OFF_ACTION);
  }

  @Test
  void shouldMapToADriverActionThatSurvivesTheOrderStartTrigger() {
    Route.Step step = transformedStep(sixHaltPoints(), 0, MOVE);

    List<PropertyAction> actions = ActionsMapping.mapPropertyActions(step.getSourcePoint());

    assertSingleDriverAction(actions, LASER_ON_ACTION);
    assertTrue(actions.get(0).getTrigger().contains(ActionTrigger.ORDER_START));
    assertTrue(actions.get(0).getTrigger().contains(ActionTrigger.ORDER_END));
    assertTrue(actions.get(0).getTrigger().contains(ActionTrigger.PASSING));
  }

  private static void assertSingleDriverAction(
      List<PropertyAction> actions,
      String expectedActionType
  ) {
    assertEquals(1, actions.size());
    assertEquals(expectedActionType, actions.get(0).getActionType());
    assertEquals(BlockingType.NONE, actions.get(0).getBlockingType());
  }

  private Route.Step transformedStep(
      List<Point> points,
      int routeIndex,
      String finalOperation
  ) {
    return transformer.apply(command(routeOf(points), routeIndex, finalOperation)).getStep();
  }

  private Route.Step transformedApproachStep(List<Point> points, int routeIndex) {
    return transformer
        .apply(command(routeOf(points), routeIndex, NOP, Map.of(LEG_PROPERTY, APPROACH_LEG)))
        .getStep();
  }

  private static String laserAction(Point point) {
    return point.getProperty(LASER_ACTION_PROPERTY);
  }

  private static String laserAction(Path path) {
    return path.getProperty(LASER_ACTION_PROPERTY);
  }

  private static List<Point> sixHaltPoints() {
    List<Point> points = new ArrayList<>();
    for (String name : List.of("A", "B", "C", "D", "E", "F")) {
      points.add(new Point(name).withType(Point.Type.HALT_POSITION));
    }
    return points;
  }

  private static Route routeOf(List<Point> points) {
    List<Route.Step> steps = new ArrayList<>();
    for (int i = 1; i < points.size(); i++) {
      steps.add(
          stepBetween(points.get(i - 1), points.get(i), Vehicle.Orientation.FORWARD, i - 1));
    }
    return new Route(steps);
  }

  private static Route.Step stepBetween(
      Point source,
      Point destination,
      Vehicle.Orientation orientation,
      int routeIndex
  ) {
    Path path = new Path(
        source.getName() + " --- " + destination.getName(),
        source.getReference(),
        destination.getReference());
    return new Route.Step(path, source, destination, orientation, routeIndex, 1);
  }

  private static long[] coordinates(long x, long y) {
    return new long[]{x, y};
  }

  private static List<Point> pointsAt(long[]... positions) {
    List<Point> points = new ArrayList<>();
    for (int i = 0; i < positions.length; i++) {
      points.add(
          new Point("P" + i)
              .withType(Point.Type.HALT_POSITION)
              .withPose(new Pose(new Triple(positions[i][0], positions[i][1], 0), Double.NaN)));
    }
    return points;
  }

  private static MovementCommand command(Route route, int routeIndex, String finalOperation) {
    return command(route, routeIndex, finalOperation, Map.of());
  }

  private static MovementCommand command(
      Route route,
      int routeIndex,
      String finalOperation,
      Map<String, String> properties
  ) {
    Route.Step step = route.getSteps().get(routeIndex);
    Point finalDestination = route.getFinalDestinationPoint();

    return new MovementCommand(
        new TransportOrder("transportOrder", List.of()),
        driveOrderFor(route, finalDestination),
        step,
        MOVE,
        null,
        routeIndex == route.getSteps().size() - 1,
        null,
        finalDestination,
        finalOperation,
        properties);
  }

  private static MovementCommand stationaryCommand(String finalOperation) {
    Point destination = new Point("A").withType(Point.Type.HALT_POSITION);
    Route.Step step
        = new Route.Step(null, null, destination, Vehicle.Orientation.UNDEFINED, 0, 0);
    Route route = new Route(List.of(step));

    return new MovementCommand(
        new TransportOrder("transportOrder", List.of()),
        driveOrderFor(route, destination),
        step,
        MOVE,
        null,
        true,
        null,
        destination,
        finalOperation,
        Map.of());
  }

  private static DriveOrder driveOrderFor(Route route, Point finalDestination) {
    return new DriveOrder("order", new DriveOrder.Destination(finalDestination.getReference()))
        .withRoute(route);
  }
}
