package org.opentcs.fms.kernel;

import static org.assertj.core.api.Assertions.assertThat;

import java.time.Instant;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.DriveOrder;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.fms.kernel.mapf.ClaimPolicy;
import org.opentcs.fms.kernel.scheduling.RetreatHolds;

class ClaimPoliciesTest {

  private static TransportOrder order(String name, long createdAtMillis) {
    return new TransportOrder(name, List.of())
        .withCreationTime(Instant.ofEpochMilli(createdAtMillis));
  }

  private static TransportOrder dropOffOrder(String name, long createdAtMillis) {
    return order(name, createdAtMillis)
        .withProperty(RetreatHolds.LEG_PROPERTY, RetreatHolds.DROPOFF_LEG);
  }

  private static TransportOrder dropOffWithRetreat(String name, long createdAtMillis) {
    List<DriveOrder> legs = List.of(
        new DriveOrder(
            "unload",
            new DriveOrder.Destination(new Point("3180").getReference())
                .withOperation("liftDown")),
        new DriveOrder(
            "move-3179",
            new DriveOrder.Destination(new Point("3179").getReference())
                .withOperation(DriveOrder.Destination.OP_MOVE)));
    return new TransportOrder(name, legs)
        .withCreationTime(Instant.ofEpochMilli(createdAtMillis))
        .withProperty(RetreatHolds.LEG_PROPERTY, RetreatHolds.DROPOFF_LEG);
  }

  @Test
  void claimRankIsTheOrderCreationTimeInEpochMillis() {
    assertThat(ClaimPolicies.claimRankOf(order("TO-1", 1_725_000_000_123L)))
        .as("GoalClaimer sorts on this value, so an older order must yield a strictly smaller"
            + " rank; anything but epoch millis silently reorders the whole fleet")
        .isEqualTo(1_725_000_000_123L);
  }

  @Test
  void orderCreatedEarlierRanksBeforeOrderCreatedLater() {
    assertThat(ClaimPolicies.claimRankOf(order("TO-old", 100L)))
        .isLessThan(ClaimPolicies.claimRankOf(order("TO-new", 200L)));
  }




  @Test
  void policyCarriesRankAndAxisPerVehicle() {
    Vehicle dropping = new Vehicle("V07");
    Vehicle hauling = new Vehicle("V09");
    Map<String, TransportOrder> orders = Map.of(
        "V07", dropOffOrder("TO-drop", 100L),
        "V09", order("TO-haul", 200L));

    ClaimPolicy policy = ClaimPolicies.of(
        List.of(dropping, hauling),
        vehicle -> Optional.ofNullable(orders.get(vehicle.getName())));

    assertThat(policy.rank()).containsExactlyInAnyOrderEntriesOf(Map.of("V07", 100L, "V09", 200L));
  }

  @Test
  void vehicleWithoutAnOrderIsAbsentFromTheRankAndClaimsLast() {
    Vehicle idle = new Vehicle("V01");

    ClaimPolicy policy = ClaimPolicies.of(List.of(idle), vehicle -> Optional.empty());

    assertThat(policy.rank())
        .as("GoalClaimer falls back to Long.MAX_VALUE for absent vehicles, which is what makes"
            + " a parked vehicle claim after every vehicle that is actually working")
        .doesNotContainKey("V01");
  }

  @Test
  void policyMarksTheVehicleThatIsBackingOutOfItsSlot() {
    Vehicle retreating = new Vehicle("V07");
    Vehicle arriving = new Vehicle("V09");
    Map<String, TransportOrder> orders = Map.of(
        "V07", dropOffWithRetreat("TO-drop", 300L).withCurrentDriveOrderIndex(1),
        "V09", dropOffWithRetreat("TO-haul", 100L).withCurrentDriveOrderIndex(0));

    ClaimPolicy policy = ClaimPolicies.of(
        List.of(retreating, arriving),
        vehicle -> Optional.ofNullable(orders.get(vehicle.getName())));

    assertThat(policy.isRetreating("V07")).isTrue();
    assertThat(policy.isRetreating("V09"))
        .as("V09 has not unloaded yet, so it is queueing into the aisle, not backing out of it")
        .isFalse();
  }
}
