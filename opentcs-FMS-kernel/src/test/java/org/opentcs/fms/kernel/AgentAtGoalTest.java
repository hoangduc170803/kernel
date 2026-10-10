package org.opentcs.fms.kernel;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.entry;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Map;
import java.util.Set;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.DriveOrder;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.fms.kernel.AgentAtGoal.Arrival;
import org.opentcs.fms.kernel.scheduling.RetreatHolds;

class AgentAtGoalTest {

  private static final Predicate<String> PARK_POSITIONS = Set.of("1001", "1002")::contains;

  @Test
  void aVehicleThatFinishedBackingOutOfTheSlotHasReachedItsGoal() {
    TransportOrder retreat = dropOff("3179", "3178").withCurrentDriveOrderIndex(1);

    assertThat(AgentAtGoal.reached(
        idle("V1"), Optional.of(retreat), "3178", "3178", PARK_POSITIONS))
        .isEqualTo(Arrival.RETREATED);
  }

  @Test
  void aVehicleStillBackingOutHasNotReachedItsGoalYet() {
    TransportOrder retreat = dropOff("3179", "3178").withCurrentDriveOrderIndex(1);

    assertThat(AgentAtGoal.reached(
        idle("V1"), Optional.of(retreat), "3179", "3178", PARK_POSITIONS))
        .isNull();
  }

  @Test
  void unloadingIsNotAnArrivalBecauseTheRetreatHasNotHappened() {
    TransportOrder unloading = dropOff("3179", "3178").withCurrentDriveOrderIndex(0);

    assertThat(AgentAtGoal.reached(
        idle("V1"), Optional.of(unloading), "3180", "3180", PARK_POSITIONS))
        .isNull();
  }

  @Test
  void standingOnAParkPositionWithNothingLeftToDoIsAnArrival() {
    assertThat(AgentAtGoal.reached(idle("V1"), Optional.empty(), "1001", "1001", PARK_POSITIONS))
        .isEqualTo(Arrival.PARKED);
  }

  @Test
  void drivingThroughAParkPositionIsNotAnArrival() {
    assertThat(AgentAtGoal.reached(idle("V1"), Optional.empty(), "1001", "0091", PARK_POSITIONS))
        .isNull();
  }

  @Test
  void chargingCountsWhereverTheChargerIsBecauseChargePointsAreNotParkPositions() {
    Vehicle charging = new Vehicle("V1").withState(Vehicle.State.CHARGING);

    assertThat(AgentAtGoal.reached(charging, Optional.empty(), "1007", "1007", PARK_POSITIONS))
        .isEqualTo(Arrival.CHARGING);
  }

  @Test
  void anIdleVehicleStandingAnywhereElseHasNotReachedAnyGoal() {
    assertThat(AgentAtGoal.reached(idle("V1"), Optional.empty(), "0084", "0084", PARK_POSITIONS))
        .as("no order is not the same as served; this is the reset that used to fire on every"
            + " vehicle waiting between two transport orders of one task")
        .isNull();
  }

  @Test
  void aVehicleHeadingSomewhereElseHasNotReachedAnyGoal() {
    TransportOrder pickup = new TransportOrder("PICKUP-V1", List.of(move("2701")));

    assertThat(AgentAtGoal.reached(
        idle("V1"), Optional.of(pickup), "0084", "2701", PARK_POSITIONS))
        .isNull();
  }

  @Test
  void theFleetWideScanNamesOnlyTheVehiclesThatArrived() {
    Vehicle parked = idle("V1");
    Vehicle waiting = idle("V2");
    Vehicle charging = new Vehicle("V3").withState(Vehicle.State.CHARGING);

    Map<String, Arrival> arrived = AgentAtGoal.among(
        List.of(parked, waiting, charging),
        vehicle -> Optional.empty(),
        Map.of("V1", "1001", "V2", "0084", "V3", "1007"),
        Map.of("V1", "1001", "V2", "0084", "V3", "1007"),
        PARK_POSITIONS);

    assertThat(arrived).containsOnly(
        entry("V1", Arrival.PARKED), entry("V3", Arrival.CHARGING));
  }

  @Test
  void aVehicleWithoutAKnownPositionIsNeverCountedAsArrived() {
    assertThat(AgentAtGoal.reached(idle("V1"), Optional.empty(), null, null, PARK_POSITIONS))
        .isNull();
  }

  private static Vehicle idle(String name) {
    return new Vehicle(name).withState(Vehicle.State.IDLE);
  }

  private static TransportOrder dropOff(String... retreatCells) {
    List<DriveOrder> driveOrders = new ArrayList<>();
    driveOrders.add(new DriveOrder(
        "unload",
        new DriveOrder.Destination(new Point("3180").getReference())
            .withOperation("liftDown")));
    for (String cell : retreatCells) {
      driveOrders.add(move(cell));
    }
    return new TransportOrder("DROPOFF-V1", driveOrders)
        .withProperty(RetreatHolds.LEG_PROPERTY, RetreatHolds.DROPOFF_LEG);
  }

  private static DriveOrder move(String point) {
    return new DriveOrder(
        "move-" + point,
        new DriveOrder.Destination(new Point(point).getReference())
            .withOperation(DriveOrder.Destination.OP_MOVE));
  }
}
