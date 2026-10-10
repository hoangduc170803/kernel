package org.opentcs.fms.kernel;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Test;
import org.opentcs.components.kernel.services.InternalTCSObjectService;
import org.opentcs.components.kernel.services.InternalVehicleService;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.DriveOrder;
import org.opentcs.data.order.Route;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.fms.kernel.rerouting.CommittedRouteFrontier;
import org.opentcs.fms.kernel.scheduling.RetreatHolds;
import org.opentcs.strategies.basic.dispatching.priorization.CompositeOrderComparator;
import org.opentcs.strategies.basic.dispatching.rerouting.VehiclePositionResolver;
import org.opentcs.strategies.basic.dispatching.selection.vehicles.IsAvailableForAnyOrder;

class FleetSnapshotServiceTest {

  private static final Point STANDING_ON = new Point("1000");
  private static final Point CHARGE_POINT = new Point("1001");
  private static final Point PICKUP_POINT = new Point("1002");

  @Test
  void anOrderTheVehicleCannotTakeYetGrantsNoGoal() {
    Vehicle waiting = vehicleAt("V6");

    assertThat(read(waiting, false, orderTo(waiting, CHARGE_POINT,
                                            TransportOrder.State.DISPATCHABLE)).goals())
        .as("openTCS issues no movement command for an order it cannot assign, so planning a route"
            + " towards its destination invents a trip that will never be executed - and that"
            + " phantom route takes precedence over vehicles that really are moving")
        .containsEntry("V6", "1000");
  }

  @Test
  void anOrderTheVehicleIsAboutToAcceptGrantsItsGoal() {
    Vehicle waiting = vehicleAt("V6");

    assertThat(read(waiting, true, orderTo(waiting, CHARGE_POINT,
                                           TransportOrder.State.DISPATCHABLE)).goals())
        .as("the dispatcher assigns this order within the same cycle, so the goal is real and"
            + " announcing it early is what lets the fleet plan cover the very first route")
        .containsEntry("V6", "1001");
  }

  @Test
  void anOrderTheDispatcherHasNotPickedUpYetGrantsItsGoal() {
    Vehicle waiting = vehicleAt("V6");

    assertThat(read(waiting, true, orderTo(waiting, CHARGE_POINT,
                                           TransportOrder.State.RAW)).goals())
        .as("a new order sits in RAW until CheckNewOrdersPhase activates it, and that phase runs"
            + " inside super.dispatch() - after the fleet snapshot - so RAW is the state the"
            + " snapshot actually sees; its destinations are complete from construction on")
        .containsEntry("V6", "1001");
  }

  @Test
  void aPendingDropOffOrderSuppliesTheSameOrderToGoalAndClaim() {
    Vehicle waiting = vehicleAt("V6");
    TransportOrder dropOff = orderTo(waiting, CHARGE_POINT, TransportOrder.State.DISPATCHABLE)
        .withProperty(RetreatHolds.LEG_PROPERTY, RetreatHolds.DROPOFF_LEG);

    FleetSnapshotService.Fleet fleet = read(waiting, true, dropOff);

    assertThat(fleet.goals()).containsEntry("V6", "1001");
    assertThat(fleet.claimPolicy().isDroppingOff("V6")).isTrue();
    assertThat(fleet.claimPolicy().isOnDropOffLeg("V6")).isTrue();
    assertThat(fleet.claimPolicy().rank()).containsKey("V6");
  }

  @Test
  void anOrderWaitingOnItsDependenciesGrantsItsGoal() {
    Vehicle waiting = vehicleAt("V6");

    assertThat(read(waiting, true, orderTo(waiting, CHARGE_POINT,
                                           TransportOrder.State.ACTIVE)).goals())
        .containsEntry("V6", "1001");
  }

  @Test
  void anAcceptedOrderGrantsItsGoalEvenBeforeARouteExists() {
    Vehicle driving = vehicleAt("V6");

    assertThat(read(driving, false, orderTo(driving, CHARGE_POINT,
                                            TransportOrder.State.BEING_PROCESSED)).goals())
        .as("between assignment and the first computed route the drive order carries no steps, so"
            + " this fallback is the only source of the goal; losing it would stall the vehicle")
        .containsEntry("V6", "1001");
  }

  @Test
  void aVehicleWaitingForDropOffStandsStillUntilTheDropOffOrderExists() {
    Vehicle waiting = vehicleAt("V6").withProperty(WaitDropOffListener.WAIT_DROPOFF_PROPERTY, "true");

    assertThat(read(waiting, true).goals())
        .as("the drop-off barrier: after the approach leg the vehicle must not wander off before"
            + " WES hands it the drop-off it is standing by for")
        .containsEntry("V6", "1000");
  }

  @Test
  void aVehicleWaitingForDropOffLeavesOnceItsDropOffOrderIsPinnedToIt() {
    Vehicle waiting = vehicleAt("V6").withProperty(WaitDropOffListener.WAIT_DROPOFF_PROPERTY, "true");

    assertThat(read(waiting, true, orderTo(waiting, CHARGE_POINT,
                                           TransportOrder.State.DISPATCHABLE)).goals())
        .as("the wait is over the moment the drop-off exists; pinning the vehicle any longer"
            + " leaves it without a goal, so the fleet plan holds it in place and the very order"
            + " it is waiting for can never be routed")
        .containsEntry("V6", "1001");
  }

  @Test
  void aFinishedOrderGrantsNoGoal() {
    Vehicle done = vehicleAt("V6");

    assertThat(read(done, true, orderTo(done, CHARGE_POINT, TransportOrder.State.FINISHED)).goals())
        .containsEntry("V6", "1000");
  }

  @Test
  void aWithdrawnOrderGrantsNoGoal() {
    Vehicle abandoning = vehicleAt("V6");

    assertThat(read(abandoning, true, orderTo(abandoning, CHARGE_POINT,
                                              TransportOrder.State.WITHDRAWN)).goals())
        .as("WITHDRAWN is not a final state, but its destination has been abandoned - aiming the"
            + " fleet plan at it would drive the vehicle somewhere nobody asked for")
        .containsEntry("V6", "1000");
  }

  @Test
  void aWithdrawnOrderStillPinnedToTheVehicleGrantsNoGoal() {
    Vehicle carrier = vehicleAt("V6");
    TransportOrder abandoned =
        routed(carrier, TransportOrder.State.WITHDRAWN, STANDING_ON, PICKUP_POINT);

    assertThat(read(pinned(carrier, abandoned), true, abandoned).goals())
        .as("openTCS leaves a withdrawn order pinned to the vehicle until the withdrawal reaches a"
            + " final state - 1.9s in production - and reading the destination off its route"
            + " during that window aims two vehicles at one cell, which has no solution")
        .containsEntry("V6", "1000");
  }

  @Test
  void anOrderBeingProcessedPinnedToTheVehicleGrantsTheEndOfItsRoute() {
    Vehicle carrier = vehicleAt("V6");
    TransportOrder driving =
        routed(carrier, TransportOrder.State.BEING_PROCESSED, STANDING_ON, PICKUP_POINT);

    assertThat(read(pinned(carrier, driving), false, driving).goals())
        .containsEntry("V6", "1002");
  }

  @Test
  void aDispatchableOrderPinnedToTheVehicleStillGrantsItsGoal() {
    Vehicle carrier = vehicleAt("V6");
    TransportOrder accepted =
        routed(carrier, TransportOrder.State.DISPATCHABLE, STANDING_ON, PICKUP_POINT);

    assertThat(read(pinned(carrier, accepted), false, accepted).goals())
        .as("a vehicle the dispatcher has not released reaches its goal through the pinned order"
            + " alone; narrowing the pinned rung to BEING_PROCESSED leaves every other test in"
            + " this class green because they all read the goal off the claimed-order rung")
        .containsEntry("V6", "1002");
  }

  @Test
  void aWithdrawnOrderStillReservesTheCellsTheVehicleIsCrossing() {
    Vehicle carrier = vehicleAt("V6");
    TransportOrder abandoned =
        routed(carrier, TransportOrder.State.WITHDRAWN, STANDING_ON, PICKUP_POINT);

    assertThat(read(pinned(carrier, abandoned), true, 0, abandoned).committedPrefixes())
        .as("a regular withdrawal lets the vehicle finish the commands openTCS already handed it,"
            + " so the cells it is still crossing stay reserved even though its order no longer"
            + " grants a goal - dropping them lets another vehicle plan into the in-flight edge")
        .containsEntry("V6", List.of("1000", "1002"));
  }

  @Test
  void keepsTheCommittedPrefixWhenTheRouteRevisitsTheStandingPoint() {
    Vehicle carrier = vehicleAt("V6");
    TransportOrder driving = routed(carrier, TransportOrder.State.BEING_PROCESSED,
                                    STANDING_ON, CHARGE_POINT, STANDING_ON, PICKUP_POINT);

    assertThat(read(pinned(carrier, driving), false, 1, driving).committedPrefixes())
        .containsEntry("V6", List.of("1000", "1001", "1000"));
  }

  @Test
  void returnsNoCommittedPrefixWhenNothingIsInFlight() {
    Vehicle carrier = vehicleAt("V6");
    TransportOrder driving = routed(carrier, TransportOrder.State.BEING_PROCESSED,
                                    STANDING_ON, CHARGE_POINT, STANDING_ON, PICKUP_POINT);

    assertThat(read(pinned(carrier, driving), false, driving).committedPrefixes())
        .doesNotContainKey("V6");
  }

  @Test
  void picksTheOrderTheDispatcherWouldAssignFirst() {
    Vehicle queued = vehicleAt("V6");

    assertThat(read(queued, true,
                    orderTo(queued, PICKUP_POINT, TransportOrder.State.DISPATCHABLE, "B-later"),
                    orderTo(queued, CHARGE_POINT, TransportOrder.State.DISPATCHABLE, "A-first"))
                   .goals())
        .as("a vehicle can have several orders waiting for it; picking by iteration order would"
            + " hand the fleet plan a different goal on every kernel start")
        .containsEntry("V6", "1001");
  }

  @Test
  void aPausedVehicleIsPinnedWhereItStandsEvenWithAnOrderInFlight() {
    Vehicle held = vehicleAt("V6").withPaused(true);

    FleetSnapshotService.Fleet fleet = read(held, true,
                                            orderTo(held, CHARGE_POINT,
                                                    TransportOrder.State.BEING_PROCESSED));

    assertThat(fleet.goals())
        .as("PausedVehicleModule refuses every allocation a paused vehicle asks for, so planning"
            + " it towards its destination promises a move openTCS will never let it make, and"
            + " routes the rest of the fleet through a cell that never clears")
        .containsEntry("V6", "1000");
    assertThat(fleet.immobile()).contains("V6");
  }

  @Test
  void aPausedVehicleWithoutAnOrderIsImmobileRatherThanMerelyStandingAtItsGoal() {
    Vehicle idle = vehicleAt("V6").withPaused(true);

    assertThat(read(idle, true).immobile())
        .as("having its goal on the point it occupies already keeps it still, but only membership"
            + " of the immobile set makes unpausing change what ReplanPolicy compares against")
        .contains("V6");
  }

  private static Vehicle vehicleAt(String name) {
    return new Vehicle(name).withCurrentPosition(STANDING_ON.getReference());
  }

  private static Vehicle pinned(Vehicle vehicle, TransportOrder order) {
    return vehicle.withTransportOrder(order.getReference());
  }

  private static TransportOrder routed(
      Vehicle vehicle, TransportOrder.State state, Point... points) {
    List<Route.Step> steps = new ArrayList<>();
    for (int i = 1; i < points.length; i++) {
      steps.add(
          new Route.Step(null, points[i - 1], points[i], Vehicle.Orientation.FORWARD, i - 1, 1));
    }
    Point destination = points[points.length - 1];
    DriveOrder driveOrder = new DriveOrder(
        "drive-order-0",
        new DriveOrder.Destination(destination.getReference())
            .withOperation(DriveOrder.Destination.OP_MOVE))
        .withRoute(new Route(steps));
    return new TransportOrder("ORDER-" + vehicle.getName(), List.of(driveOrder))
        .withIntendedVehicle(vehicle.getReference())
        .withCurrentDriveOrderIndex(0)
        .withState(state);
  }

  private static TransportOrder orderTo(Vehicle vehicle, Point destination,
                                        TransportOrder.State state) {
    return orderTo(vehicle, destination, state, "ORDER-" + vehicle.getName());
  }

  private static TransportOrder orderTo(Vehicle vehicle, Point destination,
                                        TransportOrder.State state, String name) {
    DriveOrder driveOrder = new DriveOrder(
        "drive-order-0",
        new DriveOrder.Destination(destination.getReference()).withOperation("startCharging"));
    return new TransportOrder(name, List.of(driveOrder))
        .withIntendedVehicle(vehicle.getReference())
        .withState(state);
  }

  private static FleetSnapshotService.Fleet read(Vehicle vehicle, boolean availableForOrders,
                                                 TransportOrder... orders) {
    return read(vehicle, availableForOrders, null, orders);
  }

  private static FleetSnapshotService.Fleet read(Vehicle vehicle, boolean availableForOrders,
                                                 Integer frozenStepIndex,
                                                 TransportOrder... orders) {
    InternalVehicleService vehicleService = mock(InternalVehicleService.class);
    when(vehicleService.fetch(Vehicle.class)).thenReturn(Set.of(vehicle));
    when(vehicleService.fetch(TransportOrder.class))
        .thenReturn(Arrays.stream(orders).collect(Collectors.toSet()));
    when(vehicleService.fetch(eq(TransportOrder.class), anyString()))
        .thenAnswer(invocation -> Arrays.stream(orders)
            .filter(order -> order.getName().equals(invocation.<String>getArgument(1)))
            .findFirst());

    IsAvailableForAnyOrder available = mock(IsAvailableForAnyOrder.class);
    when(available.test(any())).thenReturn(availableForOrders);

    CompositeOrderComparator byName = mock(CompositeOrderComparator.class);
    when(byName.compare(any(), any())).thenAnswer(
        invocation -> Comparator.comparing(TransportOrder::getName)
            .compare(invocation.getArgument(0), invocation.getArgument(1)));

    VehiclePositionResolver positions = mock(VehiclePositionResolver.class);
    CommittedRouteFrontier committedRouteFrontier = mock(CommittedRouteFrontier.class);
    when(committedRouteFrontier.frozenStepIndex(eq(vehicle), anyInt())).thenAnswer(
        invocation -> frozenStepIndex == null
            ? invocation.getArgument(1)
            : frozenStepIndex);

    return new FleetSnapshotService(
        vehicleService,
        mock(InternalTCSObjectService.class),
        positions,
        byName,
        available,
        committedRouteFrontier).read();
  }
}
