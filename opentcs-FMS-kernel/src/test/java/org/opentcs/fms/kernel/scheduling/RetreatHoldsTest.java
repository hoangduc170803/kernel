package org.opentcs.fms.kernel.scheduling;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.opentcs.data.model.Point;
import org.opentcs.data.order.DriveOrder;
import org.opentcs.data.order.TransportOrder;

class RetreatHoldsTest {

  @Test
  void aVehicleStillCarryingTheLoadIsNotRetreatingYet() {
    TransportOrder order = dropOff("3179", "3178").withCurrentDriveOrderIndex(0);

    assertThat(RetreatHolds.isRetreating(order))
        .as("the first drive order is the unload itself; treating it as a retreat would hand the"
            + " top priority tier to every vehicle merely heading for a slot")
        .isFalse();
  }

  @Test
  void aVehicleBackingOutOfTheSlotIsRetreating() {
    TransportOrder order = dropOff("3179", "3178").withCurrentDriveOrderIndex(1);

    assertThat(RetreatHolds.isRetreating(order)).isTrue();
  }

  @Test
  void anOrderThatIsNotADropOffLegNeverCountsAsARetreat() {
    TransportOrder park = new TransportOrder("PARK-V1", List.of(move("0091")))
        .withCurrentDriveOrderIndex(0);

    assertThat(RetreatHolds.isRetreating(park)).isFalse();
  }

  @Test
  void onlyAnOrderCarryingTheLegPropertyIsADropOffLeg() {
    assertThat(RetreatHolds.isDropOffLeg(dropOff("3179"))).isTrue();
    assertThat(RetreatHolds.isDropOffLeg(new TransportOrder("PARK-V1", List.of(move("0091")))))
        .isFalse();
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
