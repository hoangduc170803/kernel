package org.opentcs.fms.kernel.vda5050;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.opentcs.data.model.Path;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Pose;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.DriveOrder;
import org.opentcs.data.order.Route;
import org.opentcs.drivers.vehicle.MovementCommand;

class AubotOrderContextTest {

  private static final double LOADED_SPEED = 40.0;
  private static final double UNLOADED_SPEED = 70.0;
  private static final double TURN_SPEED = 12.0;
  private static final AubotSpeeds SPEEDS
      = new AubotSpeeds(LOADED_SPEED, UNLOADED_SPEED, TURN_SPEED);

  private static AubotOrderContext context() {
    return new AubotOrderContext(SPEEDS);
  }

  private static Point pointAt(String name, long x, long y) {
    return new Point(name).withPose(new Pose(new Triple(x, y, 0), 0.0));
  }

  private static MovementCommand commandAlong(Point source, Point destination,
                                              Vehicle.Orientation orientation) {
    return commandAlong(source, destination, orientation, 1000);
  }

  private static MovementCommand outAndBack(Point corridor, Point pocket) {
    Path path = new Path(
        corridor.getName() + " --- " + pocket.getName(),
        corridor.getReference(),
        pocket.getReference())
        .withMaxVelocity(1000)
        .withMaxReverseVelocity(1000);
    Route route = new Route(List.of(
        new Route.Step(path, corridor, pocket, Vehicle.Orientation.FORWARD, 0, 1),
        new Route.Step(path, pocket, corridor, Vehicle.Orientation.BACKWARD, 1, 1)));
    DriveOrder driveOrder = mock(DriveOrder.class);
    when(driveOrder.getRoute()).thenReturn(route);
    MovementCommand command = mock(MovementCommand.class);
    when(command.getDriveOrder()).thenReturn(driveOrder);
    return command;
  }

  private static MovementCommand commandAlong(Point source, Point destination,
                                              Vehicle.Orientation orientation, int maxVelocity) {
    Path path = new Path(
        source.getName() + " --- " + destination.getName(),
        source.getReference(),
        destination.getReference())
        .withMaxVelocity(maxVelocity);
    Route route = new Route(
        List.of(new Route.Step(path, source, destination, orientation, 0, 1)));
    DriveOrder driveOrder = mock(DriveOrder.class);
    when(driveOrder.getRoute()).thenReturn(route);
    MovementCommand command = mock(MovementCommand.class);
    when(command.getDriveOrder()).thenReturn(driveOrder);
    return command;
  }

  private static MovementCommand commandThrough(Vehicle.Orientation orientation,
                                                Point... waypoints) {
    List<Route.Step> steps = new ArrayList<>();
    for (int index = 0; index < waypoints.length - 1; index++) {
      Point source = waypoints[index];
      Point destination = waypoints[index + 1];
      Path path = new Path(
          source.getName() + " --- " + destination.getName(),
          source.getReference(),
          destination.getReference())
          .withMaxVelocity(1000);
      steps.add(new Route.Step(path, source, destination, orientation, index, 1));
    }
    DriveOrder driveOrder = mock(DriveOrder.class);
    when(driveOrder.getRoute()).thenReturn(new Route(steps));
    MovementCommand command = mock(MovementCommand.class);
    when(command.getDriveOrder()).thenReturn(driveOrder);
    return command;
  }

  @Test
  void compactsPathNameToFirstAndLastFourCharacters() {
    assertThat(context().compactEdgeId("0004 --- 0005")).isEqualTo("00040005");
  }

  @Test
  void restoresEveryCompactedEdgeId() {
    AubotOrderContext context = context();
    String[] pathNames = {
      "0004 --- 0005",
      "3049 --- 3057",
      "0270 --- 0280",
      "B300 --- B301"
    };

    for (String pathName : pathNames) {
      assertThat(context.expandEdgeId(context.compactEdgeId(pathName))).isEqualTo(pathName);
    }
  }

  @Test
  void leavesUnknownCompactEdgeIdUntouched() {
    assertThat(context().expandEdgeId("99998888")).isEqualTo("99998888");
  }

  @Test
  void leavesShortEdgeIdUntouched() {
    assertThat(context().compactEdgeId("00040005")).isEqualTo("00040005");
  }

  @Test
  void pathNamesSharingBothAffixesCollapseOntoTheSameCompactId() {
    AubotOrderContext context = context();

    String first = context.compactEdgeId("0004 --- 0005");
    String second = context.compactEdgeId("0004 xxx 0005");

    assertThat(first).isEqualTo(second);
    assertThat(context.expandEdgeId(first)).isEqualTo("0004 xxx 0005");
  }

  @Test
  void shortensDestinationActionIdToTheProductionPrefix() {
    assertThat(context().compactActionId("Order_destination_action_12")).isEqualTo("ODA_12");
  }

  @Test
  void shortensCustomActionIdToItsOwnPrefix() {
    assertThat(context().compactActionId("Order_destination_custom_action_3")).isEqualTo("ODC_3");
  }

  @Test
  void keepsEveryShortenedActionIdWithinTheVehicleLimit() {
    AubotOrderContext context = context();
    String[] actionIds = {
      "Order_destination_action_0",
      "Order_destination_action_999",
      "Order_destination_custom_action_7",
      "0004 --- 0005_action_2"
    };

    for (String actionId : actionIds) {
      assertThat(context.compactActionId(actionId).length())
          .isLessThanOrEqualTo(AubotOrderContext.MAX_ACTION_ID_LENGTH);
    }
  }

  @Test
  void restoresEveryShortenedActionId() {
    AubotOrderContext context = context();
    String[] actionIds = {
      "Order_destination_action_0",
      "Order_destination_action_999",
      "Order_destination_custom_action_7",
      "0004 --- 0005_action_2"
    };

    for (String actionId : actionIds) {
      assertThat(context.expandActionId(context.compactActionId(actionId))).isEqualTo(actionId);
    }
  }

  @Test
  void keepsConsecutiveDestinationActionIdsDistinct() {
    AubotOrderContext context = context();

    String first = context.compactActionId("Order_destination_action_1");
    String second = context.compactActionId("Order_destination_action_2");

    assertThat(first).isNotEqualTo(second);
    assertThat(context.expandActionId(first)).isEqualTo("Order_destination_action_1");
    assertThat(context.expandActionId(second)).isEqualTo("Order_destination_action_2");
  }

  @Test
  void keepsPointActionIdsOfNeighbouringPointsDistinct() {
    AubotOrderContext context = context();

    String first = context.compactActionId("1008_action_0");
    String second = context.compactActionId("2008_action_0");
    String third = context.compactActionId("1007_action_0");

    assertThat(List.of(first, second, third)).doesNotHaveDuplicates();
    assertThat(context.expandActionId(first)).isEqualTo("1008_action_0");
    assertThat(context.expandActionId(second)).isEqualTo("2008_action_0");
    assertThat(context.expandActionId(third)).isEqualTo("1007_action_0");
  }

  @Test
  void keepsPathActionIdWithinTheVehicleLimit() {
    AubotOrderContext context = context();

    String compacted = context.compactActionId("0004 --- 0005_action_2");

    assertThat(compacted.length()).isLessThanOrEqualTo(AubotOrderContext.MAX_ACTION_ID_LENGTH);
    assertThat(context.expandActionId(compacted)).isEqualTo("0004 --- 0005_action_2");
  }

  @Test
  void leavesShortActionIdUntouched() {
    assertThat(context().compactActionId("ODA_1")).isEqualTo("ODA_1");
  }

  @Test
  void keepsShortenedOrderIdWithinTheVehicleLimit() {
    AubotOrderContext context = context();
    String[] orderIds = {
      "PARK-B300_3_6-0377-5bbb8bf6-7fc2-4df4-a22a-e25f9c20e8b0-0",
      "PICKUP-B300_3_6-0009-0d4dd3a5-bfe5-4d90-8893-31ed8d12cae5-1",
      "TOrder-0001-0"
    };

    for (String orderId : orderIds) {
      assertThat(context.compactOrderId(orderId).length())
          .isLessThanOrEqualTo(AubotOrderContext.MAX_ORDER_ID_LENGTH);
    }
  }

  @Test
  void keepsTheDriveOrderIndexInTheShortenedOrderId() {
    String orderId = "PARK-B300_3_6-0377-5bbb8bf6-7fc2-4df4-a22a-e25f9c20e8b0-12";

    assertThat(context().compactOrderId(orderId)).endsWith("-12");
  }

  @Test
  void restoresEveryShortenedOrderId() {
    AubotOrderContext context = context();
    String[] orderIds = {
      "PARK-B300_3_6-0377-5bbb8bf6-7fc2-4df4-a22a-e25f9c20e8b0-0",
      "PICKUP-B300_3_6-0009-0d4dd3a5-bfe5-4d90-8893-31ed8d12cae5-1",
      "TOrder-0001-0"
    };

    for (String orderId : orderIds) {
      assertThat(context.expandOrderId(context.compactOrderId(orderId))).isEqualTo(orderId);
    }
  }

  @Test
  void keepsDriveOrdersOfTheSameTransportOrderDistinct() {
    AubotOrderContext context = context();
    String transportOrder = "PARK-B300_3_6-0377-5bbb8bf6-7fc2-4df4-a22a-e25f9c20e8b0";

    String first = context.compactOrderId(transportOrder + "-0");
    String second = context.compactOrderId(transportOrder + "-1");

    assertThat(first).isNotEqualTo(second);
    assertThat(context.expandOrderId(first)).isEqualTo(transportOrder + "-0");
    assertThat(context.expandOrderId(second)).isEqualTo(transportOrder + "-1");
  }

  @Test
  void keepsTwoOrdersOfTheSameVehicleApart() {
    AubotOrderContext context = context();

    String first = context.compactOrderId("LOADTEST-002-B300_3_7-0");
    String second = context.compactOrderId("LOADTEST-006-B300_3_7-0");

    assertThat(first).isNotEqualTo(second);
  }

  @Test
  void keepsOrdersApartWhenOnlyTheMiddleOfTheNameDiffers() {
    AubotOrderContext context = context();
    List<String> compacted = new ArrayList<>();

    for (int index = 0; index < 50; index++) {
      compacted.add(
          context.compactOrderId("PICKUP-Vehicle-%04d-location_0160-B300_3_7-0".formatted(index))
      );
    }

    assertThat(compacted).doesNotHaveDuplicates();
  }

  @Test
  void returnsTheSameCompactOrderIdEveryTimeForOneOrder() {
    AubotOrderContext context = context();
    String orderId = "PARK-B300_3_6-0377-5bbb8bf6-7fc2-4df4-a22a-e25f9c20e8b0-0";

    assertThat(context.compactOrderId(orderId)).isEqualTo(context.compactOrderId(orderId));
  }

  @Test
  void restoresBothOriginalsOfOrdersThatUsedToCollide() {
    AubotOrderContext context = context();
    String firstOrder = "LOADTEST-002-B300_3_7-0";
    String secondOrder = "LOADTEST-006-B300_3_7-0";

    String first = context.compactOrderId(firstOrder);
    String second = context.compactOrderId(secondOrder);

    assertThat(context.expandOrderId(first)).isEqualTo(firstOrder);
    assertThat(context.expandOrderId(second)).isEqualTo(secondOrder);
  }

  @Test
  void shortensOrderIdsWithoutADriveOrderIndex() {
    AubotOrderContext context = context();
    String orderId = "PARK-B300_3_6-0377-5bbb8bf6-7fc2-4df4-a22a-e25f9c20e8b0";

    String compacted = context.compactOrderId(orderId);

    assertThat(compacted.length()).isLessThanOrEqualTo(AubotOrderContext.MAX_ORDER_ID_LENGTH);
    assertThat(context.expandOrderId(compacted)).isEqualTo(orderId);
  }

  @Test
  void leavesShortOrderIdUntouched() {
    assertThat(context().compactOrderId("586EREX-0")).isEqualTo("586EREX-0");
  }

  @Test
  void leavesUnknownCompactOrderIdUntouched() {
    assertThat(context().expandOrderId("586EREX-0")).isEqualTo("586EREX-0");
  }

  @Test
  void sendsUnloadedSpeedWhenVehicleCarriesNothing() {
    AubotOrderContext context = context();
    context.record(
        commandAlong(pointAt("0004", 0, 0), pointAt("0005", 1000, 0),
                     Vehicle.Orientation.FORWARD),
        false);

    assertThat(context.maxSpeedForEdge("0004", "0005")).contains(UNLOADED_SPEED);
  }

  @Test
  void sendsLoadedSpeedWhenVehicleCarriesCargo() {
    AubotOrderContext context = context();
    context.record(
        commandAlong(pointAt("0004", 0, 0), pointAt("0005", 1000, 0),
                     Vehicle.Orientation.FORWARD),
        true);

    assertThat(context.maxSpeedForEdge("0004", "0005")).contains(LOADED_SPEED);
  }

  @Test
  void sendsNegativeSpeedOnlyOnPathsThatForbidDrivingForward() {
    AubotOrderContext context = context();
    context.record(
        commandAlong(pointAt("0004", 0, 0), pointAt("0005", 1000, 0),
                     Vehicle.Orientation.BACKWARD, 0),
        false);

    assertThat(context.maxSpeedForEdge("0004", "0005")).contains(-UNLOADED_SPEED);
    assertThat(context.directionForEdge("0004", "0005")).contains("X-");
  }

  @Test
  void aPocketDetourKeepsBothTraversalsOfTheSamePathApart() {
    // The shape PIBT produces under congestion: duck into the pocket and come straight back out,
    // so ONE path is traversed both ways inside a single order. Driving in is forward (+), backing
    // out is reverse (-). Keying anything by path name alone lets the second write erase the first
    // and the vehicle is told to reverse into the pocket.
    AubotOrderContext context = context();
    context.record(outAndBack(pointAt("0009", 0, 0), pointAt("3073", 0, -1000)), false);

    assertThat(context.maxSpeedForEdge("0009", "3073"))
        .describedAs("driving into the pocket is forward")
        .contains(UNLOADED_SPEED);
    assertThat(context.maxSpeedForEdge("3073", "0009"))
        .describedAs("backing out of the pocket is reverse")
        .contains(-UNLOADED_SPEED);
  }

  @Test
  void slowsToTurnSpeedOnTheEdgeLeadingIntoADirectionChange() {
    AubotOrderContext context = context();
    context.record(
        commandThrough(Vehicle.Orientation.FORWARD,
                       pointAt("0004", 0, 0),
                       pointAt("0005", 1000, 0),
                       pointAt("0006", 1000, 1000)),
        false);

    assertThat(context.maxSpeedForEdge("0004", "0005")).contains(TURN_SPEED);
    assertThat(context.maxSpeedForEdge("0005", "0006")).contains(UNLOADED_SPEED);
  }

  @Test
  void keepsCruisingSpeedWhenTheRouteNeverChangesDirection() {
    AubotOrderContext context = context();
    context.record(
        commandThrough(Vehicle.Orientation.FORWARD,
                       pointAt("0004", 0, 0),
                       pointAt("0005", 1000, 0),
                       pointAt("0006", 2000, 0)),
        false);

    assertThat(context.maxSpeedForEdge("0004", "0005")).contains(UNLOADED_SPEED);
    assertThat(context.maxSpeedForEdge("0005", "0006")).contains(UNLOADED_SPEED);
  }

  @Test
  void capsApproachToATurnEvenWhenTheVehicleCarriesCargo() {
    AubotOrderContext context = context();
    context.record(
        commandThrough(Vehicle.Orientation.FORWARD,
                       pointAt("0004", 0, 0),
                       pointAt("0005", 1000, 0),
                       pointAt("0006", 1000, 1000)),
        true);

    assertThat(context.maxSpeedForEdge("0004", "0005")).contains(TURN_SPEED);
    assertThat(context.maxSpeedForEdge("0005", "0006")).contains(LOADED_SPEED);
  }

  @Test
  void sendsNegativeTurnSpeedWhenApproachingATurnInReverse() {
    AubotOrderContext context = context();
    context.record(
        commandThrough(Vehicle.Orientation.BACKWARD,
                       pointAt("0004", 0, 0),
                       pointAt("0005", 1000, 0),
                       pointAt("0006", 1000, 1000)),
        false);

    assertThat(context.maxSpeedForEdge("0004", "0005")).contains(-TURN_SPEED);
  }

  @Test
  void looksPastStepsWithoutADirectionWhenSpottingATurn() {
    AubotOrderContext context = context();
    context.record(
        commandThrough(Vehicle.Orientation.FORWARD,
                       pointAt("0004", 0, 0),
                       pointAt("0005", 1000, 0),
                       pointAt("0105", 1000, 0),
                       pointAt("0006", 1000, 1000)),
        false);

    assertThat(context.maxSpeedForEdge("0004", "0005")).contains(TURN_SPEED);
  }

  @Test
  void rotatesDirectionOntoTheFrameTheQrCodesAreStuckIn() {
    AubotOrderContext context = new AubotOrderContext(SPEEDS, AubotDirection.EAST);
    context.record(
        commandAlong(pointAt("0009", 0, 0), pointAt("0317", 0, 950),
                     Vehicle.Orientation.FORWARD),
        false);

    assertThat(context.directionForEdge("0009", "0317")).contains("X-");
    assertThat(context.maxSpeedForEdge("0009", "0317")).contains(UNLOADED_SPEED);
  }

  @Test
  void derivesAxisDirectionFromPointPositions() {
    assertThat(AubotDirection.between(pointAt("a", 0, 0), pointAt("b", 1000, 0), false))
        .isEqualTo(AubotDirection.EAST);
    assertThat(AubotDirection.between(pointAt("a", 1000, 0), pointAt("b", 0, 0), false))
        .isEqualTo(AubotDirection.WEST);
    assertThat(AubotDirection.between(pointAt("a", 0, 0), pointAt("b", 0, 1000), false))
        .isEqualTo(AubotDirection.NORTH);
    assertThat(AubotDirection.between(pointAt("a", 0, 1000), pointAt("b", 0, 0), false))
        .isEqualTo(AubotDirection.SOUTH);
  }

  @Test
  void reversesDirectionWhenVehicleDrivesBackward() {
    assertThat(AubotDirection.between(pointAt("a", 0, 0), pointAt("b", 1000, 0), true))
        .isEqualTo(AubotDirection.WEST);
    assertThat(AubotDirection.between(pointAt("a", 0, 0), pointAt("b", 0, 1000), true))
        .isEqualTo(AubotDirection.SOUTH);
  }

  @Test
  void hasNoDirectionBetweenCoincidentPoints() {
    assertThat(AubotDirection.between(pointAt("a", 500, 500), pointAt("b", 500, 500), false))
        .isNull();
  }

  @Test
  void producesCodesTheVehicleUnderstands() {
    assertThat(AubotDirection.NORTH.getCode()).isEqualTo("Y+");
    assertThat(AubotDirection.EAST.getCode()).isEqualTo("X+");
    assertThat(AubotDirection.SOUTH.getCode()).isEqualTo("Y-");
    assertThat(AubotDirection.WEST.getCode()).isEqualTo("X-");
  }
}
