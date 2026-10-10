package org.opentcs.fms.kernel.rerouting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.data.model.Path;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.DriveOrder;
import org.opentcs.data.order.Route;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.drivers.vehicle.MovementCommand;
import org.opentcs.drivers.vehicle.VehicleController;
import org.opentcs.drivers.vehicle.VehicleControllerPool;

class FmsRegularDriveOrderMergerTest {

  private FmsRegularDriveOrderMerger driveOrderMerger;
  private VehicleControllerPool vehicleControllerPool;

  @BeforeEach
  void setUp() {
    vehicleControllerPool = mock(VehicleControllerPool.class);
    driveOrderMerger = new FmsRegularDriveOrderMerger(vehicleControllerPool);
  }

  @Test
  void shouldMergeDriveOrdersWithNoRouteProgress() {
    DriveOrder orderA = createDriveOrder(1, "A", "B", "C", "D", "E", "F", "G");
    DriveOrder orderB = createDriveOrder(1, "A", "H", "I", "J", "K", "G");
    Route expected = createDriveOrder(1, "A", "H", "I", "J", "K", "G").getRoute();

    Route actual = driveOrderMerger.mergeDriveOrders(
        orderA,
        orderB,
        new TransportOrder("t1", List.of()),
        TransportOrder.ROUTE_STEP_INDEX_DEFAULT,
        new Vehicle("Vehicle")).getRoute();

    assertStepsEqualsIgnoringReroutingType(expected, actual);
  }

  @Test
  void shouldMergeDriveOrdersWithRouteProgress() {
    DriveOrder orderA = createDriveOrder(1, "A", "B", "C", "D", "E", "F", "G");
    DriveOrder orderB = createDriveOrder(1, "C", "L", "M", "N", "O", "G");
    Route expected = createDriveOrder(1, "A", "B", "C", "L", "M", "N", "O", "G").getRoute();

    Route actual = driveOrderMerger.mergeDriveOrders(
        orderA,
        orderB,
        new TransportOrder("t1", List.of()),
        1,
        new Vehicle("Vehicle")).getRoute();

    assertStepsEqualsIgnoringReroutingType(expected, actual);
  }

  @Test
  void shouldMergeDriveOrdersKeepingAlreadyTravelledLoop() {
    DriveOrder orderA = createDriveOrder(1, "A", "B", "C", "D", "E", "D", "F", "G");
    DriveOrder orderB = createDriveOrder(1, "D", "L", "M", "N", "O", "G");
    Route expected = createDriveOrder(1, "A", "B", "C", "D", "E", "D", "L", "M", "N", "O", "G").getRoute();

    Route actual = driveOrderMerger.mergeDriveOrders(
        orderA,
        orderB,
        new TransportOrder("t1", List.of()),
        4,
        new Vehicle("Vehicle")).getRoute();

    assertStepsEqualsIgnoringReroutingType(expected, actual);
  }

  @Test
  void shouldSpliceAtFirstFutureVisitDroppingLaterRevisitsOfSource() {
    DriveOrder orderA = createDriveOrder(1, "A", "B", "N", "M", "N", "M", "N", "Z");
    DriveOrder orderB = createDriveOrder(1, "N", "X", "Y", "G");
    Route expected = createDriveOrder(1, "A", "B", "N", "X", "Y", "G").getRoute();

    Route actual = driveOrderMerger.mergeDriveOrders(
        orderA,
        orderB,
        new TransportOrder("t1", List.of()),
        1,
        new Vehicle("Vehicle")).getRoute();

    assertStepsEqualsIgnoringReroutingType(expected, actual);
  }

  @Test
  void shouldNotSpliceIntoCommittedExcursionBeforeFrontier() {
    DriveOrder orderA = createDriveOrder(1, "A", "B", "C", "D", "E", "D", "E", "F", "G");
    DriveOrder orderB = createDriveOrder(1, "E", "X", "Y", "G");
    Route expected
        = createDriveOrder(1, "A", "B", "C", "D", "E", "D", "E", "X", "Y", "G").getRoute();

    Route.Step frontierStep = orderA.getRoute().getSteps().get(5);
    MovementCommand lastSent = mock(MovementCommand.class);
    when(lastSent.getStep()).thenReturn(frontierStep);
    VehicleController controller = mock(VehicleController.class);
    when(controller.getInteractionsPendingCommand()).thenReturn(Optional.empty());
    when(controller.getCommandsSent()).thenReturn(new LinkedList<>(List.of(lastSent)));
    when(vehicleControllerPool.getVehicleController("Vehicle")).thenReturn(controller);

    Route actual = driveOrderMerger.mergeDriveOrders(
        orderA,
        orderB,
        new TransportOrder("t1", List.of()),
        3,
        new Vehicle("Vehicle")).getRoute();

    assertStepsEqualsIgnoringReroutingType(expected, actual);
  }

  @Test
  void shouldDeclineRatherThanSpliceBeforeTheCommittedFrontier() {
    DriveOrder orderA = createDriveOrder(1, "A", "B", "C", "D", "E");
    DriveOrder orderB = createDriveOrder(1, "B", "X", "Y", "G");

    Route.Step frontierStep = orderA.getRoute().getSteps().get(2);
    MovementCommand lastSent = mock(MovementCommand.class);
    when(lastSent.getStep()).thenReturn(frontierStep);
    VehicleController controller = mock(VehicleController.class);
    when(controller.getInteractionsPendingCommand()).thenReturn(Optional.empty());
    when(controller.getCommandsSent()).thenReturn(new LinkedList<>(List.of(lastSent)));
    when(vehicleControllerPool.getVehicleController("Vehicle")).thenReturn(controller);

    Vehicle vehicle = new Vehicle("Vehicle");
    TransportOrder order = new TransportOrder("t1", List.of());

    assertThrows(
        IllegalStateException.class,
        () -> driveOrderMerger.mergeDriveOrders(orderA, orderB, order, 1, vehicle));
  }

  @Test
  void detectsRevisitsBeforeTheDestinationAndIgnoresWaits() {
    assertTrue(FmsRegularDriveOrderMerger.hasRevisitBeforeDestination(
        List.of("A", "B", "A", "D")));
    assertFalse(FmsRegularDriveOrderMerger.hasRevisitBeforeDestination(List.of("A", "A")));
    assertFalse(FmsRegularDriveOrderMerger.hasRevisitBeforeDestination(
        List.of("A", "B", "C", "D")));
  }

  @Test
  void detectsPlansStartingBehindSentCommitment() {
    assertTrue(FmsRegularDriveOrderMerger.planBehindCommitment("0001", "0002"));
    assertFalse(FmsRegularDriveOrderMerger.planBehindCommitment("0001", "0001"));
  }

  private DriveOrder createDriveOrder(int stepCost, String startPoint, String... pointNames) {
    List<Point> points = new ArrayList<>();
    for (String pointName : pointNames) {
      points.add(new Point(pointName));
    }
    DriveOrder.Destination dest = new DriveOrder.Destination(points.get(points.size() - 1).getReference());
    return new DriveOrder("order1", dest).withRoute(
        createRoute(new Point(startPoint), points, stepCost));
  }

  private Route createRoute(Point startPoint, List<Point> points, int costPerStep) {
    List<Route.Step> routeSteps = new ArrayList<>();
    Point srcPoint = startPoint;
    Point destPoint = points.get(0);
    Path path = new Path(
        srcPoint.getName() + " --- " + destPoint.getName(),
        srcPoint.getReference(),
        destPoint.getReference());
    routeSteps.add(
        new Route.Step(path, srcPoint, destPoint, Vehicle.Orientation.FORWARD, 0, costPerStep));

    for (int i = 1; i < points.size(); i++) {
      srcPoint = points.get(i - 1);
      destPoint = points.get(i);
      path = new Path(
          srcPoint.getName() + " --- " + destPoint.getName(),
          srcPoint.getReference(),
          destPoint.getReference());
      routeSteps.add(
          new Route.Step(path, srcPoint, destPoint, Vehicle.Orientation.FORWARD, i, costPerStep));
    }
    return new Route(routeSteps);
  }

  private void assertStepsEqualsIgnoringReroutingType(Route routeA, Route routeB) {
    assertEquals(routeA.getSteps().size(), routeB.getSteps().size());
    for (int i = 0; i < routeA.getSteps().size(); i++) {
      Route.Step stepA = routeA.getSteps().get(i);
      Route.Step stepB = routeB.getSteps().get(i);
      assertTrue(
          Objects.equals(stepA.getPath(), stepB.getPath())
              && Objects.equals(stepA.getSourcePoint(), stepB.getSourcePoint())
              && Objects.equals(stepA.getDestinationPoint(), stepB.getDestinationPoint())
              && Objects.equals(stepA.getVehicleOrientation(), stepB.getVehicleOrientation())
              && stepA.getRouteIndex() == stepB.getRouteIndex()
              && stepA.getCosts() == stepB.getCosts()
              && stepA.isExecutionAllowed() == stepB.isExecutionAllowed(),
          "Step " + i + " mismatch: expected " + stepA + " but was " + stepB);
    }
  }
}
