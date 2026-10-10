package org.opentcs.fms.kernel.integration;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.kernel.mapf.ClaimPolicy;

class SchedulerFaithfulTest {

  private SchedulerFaithfulHarness harness() throws Exception {
    File map = PlantModelXmlLoader.findMap("v7-vda5050-test.xml");
    assumeTrue(map != null, "v7-vda5050-test.xml not found; skipping");
    return new SchedulerFaithfulHarness(map);
  }

  @Test
  void rerouteClaimReplacementRefusesFreeNextStep() throws Exception {
    assertThat(harness().firstStepGrantedAfterOptionalReroute(false))
        .as("free next step is granted when the claim is left alone")
        .isTrue();
    assertThat(harness().firstStepGrantedAfterOptionalReroute(true))
        .as("free next step is refused after the claim is replaced under the in-flight allocate")
        .isFalse();
  }

  @Test
  void retreatingVehicleLeavesTheAisleSidewaysInsteadOfBackingUpIt() throws Exception {
    SchedulerFaithfulHarness h = harness();
    assertThat(h.coordinator().planSingle("V1", "3088", "3107", ClaimPolicy.DEFAULT))
        .as("both routes are 3 hops, but 3087/3097 are drop-off slots while 3108 is a plain lane")
        .containsExactly("3088", "3098", "3108", "3107");
  }

  @Test
  void waitingVehicleIsNotPlannedOffItsOwnPoint() throws Exception {
    SchedulerFaithfulHarness h = harness();
    List<String> corridor = h.coordinator().planSingle("V1", "1001", "3090", ClaimPolicy.DEFAULT);
    assumeTrue(corridor.size() >= 5, "need a corridor of at least 5 points; got " + corridor);
    String barrierPoint = corridor.get(corridor.size() / 2);

    h.addVehicle("BLOCKER", barrierPoint);
    h.addOrder("BLOCKER", barrierPoint);
    h.addWait("BLOCKER", 40);
    h.addOrder("BLOCKER", "1005");

    h.addVehicle("MOVER", "1001");
    h.addOrder("MOVER", "3090");

    boolean drained = h.run(300);

    assertThat(h.maxPlannedRouteWhileWaiting("BLOCKER"))
        .as("a waiting vehicle must be planned to stay on its single point")
        .isEqualTo(1);
    assertThat(drained)
        .as("the mover must still drain once the blocker releases its point")
        .isTrue();
  }

  @Test
  void moverReachesAGoalHeldByAWaitingVehicle() throws Exception {
    SchedulerFaithfulHarness h = harness();
    List<String> corridor = h.coordinator().planSingle("V1", "1001", "3090", ClaimPolicy.DEFAULT);
    assumeTrue(corridor.size() >= 3, "need a corridor; got " + corridor);
    String sharedGoal = corridor.get(corridor.size() - 1);

    h.addVehicle("BLOCKER", sharedGoal);
    h.addOrder("BLOCKER", sharedGoal);
    h.addWait("BLOCKER", 30);
    h.addOrder("BLOCKER", "1005");

    h.addVehicle("MOVER", "1001");
    h.addOrder("MOVER", sharedGoal);

    boolean drained = h.run(300);

    assertThat(h.maxPlannedRouteWhileWaiting("BLOCKER"))
        .as("the vehicle sitting on the shared goal must not be planned away")
        .isEqualTo(1);
    assertThat(drained)
        .as("the mover must reach the shared goal after the blocker leaves")
        .isTrue();
  }

  @Test
  void test() throws Exception {
    SchedulerFaithfulHarness h = harness();
    h.addVehicle("V1", "1000");
    h.addVehicle("V2", "1003");
    h.addOrder("V1", "1003");
    h.addOrder("V2", "1000");
    boolean drained = h.run(300);
    System.out.println("[faithful] test drained=" + drained);
    assertThat(drained).as("aaaa").isTrue();
  }

  @Test
  void twoAgentSanity() throws Exception {
    SchedulerFaithfulHarness h = harness();
    h.addVehicle("V1", "1001");
    h.addOrder("V1", "0100");
    h.addVehicle("V2", "1002");
    h.addOrder("V2", "0101");
    boolean drained = h.run(300);
    System.out.println("[faithful] twoAgent drained=" + drained);
    assertThat(drained).as("two non-conflicting agents must drain").isTrue();
  }

  @Test
  void twoAgentSanity2() throws Exception {
    SchedulerFaithfulHarness h = harness();
    h.addVehicle("V1", "1004");
    h.addOrder("V1", "1003");
    h.addVehicle("V2", "1003");
    h.addOrder("V2", "1004");
    boolean drained = h.run(300);
    System.out.println("[faithful] twoAgent drained=" + drained);
    assertThat(drained).as("two non-conflicting agents must drain").isTrue();
  }

  private static final String FOOTPRINT = "fms.pibt.footprint";

  private void withFootprint(ThrowingRunnable body) throws Exception {
    String saved = System.getProperty(FOOTPRINT);
    System.setProperty(FOOTPRINT, System.getProperty("test.footprint", "true"));
    try {
      body.run();
    }
    finally {
      if (saved == null) {
        System.clearProperty(FOOTPRINT);
      }
      else {
        System.setProperty(FOOTPRINT, saved);
      }
    }
  }

  private interface ThrowingRunnable {
    void run() throws Exception;
  }

  @Test
  void loadedVehiclesLeavingAdjacentPickupLanesStillDrain() throws Exception {
    withFootprint(() -> {
      SchedulerFaithfulHarness h = harness();
      h.addVehicle("L1", "0100");
      h.carryLoad("L1");
      h.addOrder("L1", "3090");
      h.addVehicle("L2", "0101");
      h.carryLoad("L2");
      h.addOrder("L2", "3057");
      h.addVehicle("L3", "0102");
      h.carryLoad("L3");
      h.addOrder("L3", "3089");

      boolean drained = h.run(600);
      assertThat(h.lastBodyState().loadedVehicles())
          .as("the cargo flag has to reach the solver, else the swept-disc half is inert")
          .contains("L1", "L2", "L3");
      assertThat(drained)
          .as("three loaded vehicles pivoting out of adjacent lanes must not wedge the fleet")
          .isTrue();
    });
  }

  @Test
  void theFiveAgentLoopStillDrainsWhenEveryVehicleCarriesALoad() throws Exception {
    withFootprint(() -> {
      SchedulerFaithfulHarness h = harness();
      String[][] legs = {
          {"V1", "1001", "0100", "3090", "1001"},
          {"V2", "1002", "0101", "3057", "1002"},
          {"V3", "1003", "0102", "3089", "1003"},
          {"V4", "1004", "0103", "3065", "1004"},
          {"V5", "1005", "0104", "3100", "1005"}};
      for (String[] leg : legs) {
        h.addVehicle(leg[0], leg[1]);
        h.carryLoad(leg[0]);
        h.addOrder(leg[0], leg[2]);
        h.addOrder(leg[0], leg[3]);
        h.addOrder(leg[0], leg[4]);
      }

      boolean drained = h.run(600);
      assertThat(h.lastBodyState().loadedVehicles()).hasSize(5);
      assertThat(drained)
          .as("same route set as fiveAgentRealScheduler, but every body is 565.7mm wide when"
              + " it pivots instead of 470.7mm")
          .isTrue();
    });
  }

  @Test
  void fiveAgentRealScheduler() throws Exception {
    SchedulerFaithfulHarness h = harness();
    h.addVehicle("V1", "1001");
    h.addOrder("V1", "0100");
    h.addOrder("V1", "3090");
    h.addOrder("V1", "1001");
    h.addVehicle("V2", "1002");
    h.addOrder("V2", "0101");
    h.addOrder("V2", "3057");
    h.addOrder("V2", "1002");
    h.addVehicle("V3", "1003");
    h.addOrder("V3", "0102");
    h.addOrder("V3", "3089");
    h.addOrder("V3", "1003");
    h.addVehicle("V4", "1004");
    h.addOrder("V4", "0103");
    h.addOrder("V4", "3065");
    h.addOrder("V4", "1004");
    h.addVehicle("V5", "1005");
    h.addOrder("V5", "0104");
    h.addOrder("V5", "3100");
    h.addOrder("V5", "1005");
    boolean drained = h.run(600);
    System.out.println("[faithful] fiveAgent drained=" + drained);
    assertThat(drained).as("5-agent cargo loop through the REAL scheduler").isTrue();
  }

  @Test
  void tenAgentRealScheduler() throws Exception {
    SchedulerFaithfulHarness h = harness();
    h.addVehicle("V1", "1001");
    h.addOrder("V1", "0100");
    h.addOrder("V1", "3090");
    h.addOrder("V1", "1001");
    h.addVehicle("V2", "1002");
    h.addOrder("V2", "0101");
    h.addOrder("V2", "3057");
    h.addOrder("V2", "1002");
    h.addVehicle("V3", "1003");
    h.addOrder("V3", "0102");
    h.addOrder("V3", "3089");
    h.addOrder("V3", "1003");
    h.addVehicle("V4", "1004");
    h.addOrder("V4", "0103");
    h.addOrder("V4", "3065");
    h.addOrder("V4", "1004");
    h.addVehicle("V5", "1005");
    h.addOrder("V5", "0104");
    h.addOrder("V5", "3100");
    h.addOrder("V5", "1005");
    h.addVehicle("V6", "0110");
    h.addOrder("V6", "0105");
    h.addOrder("V6", "3058");
    h.addOrder("V6", "0110");
    h.addVehicle("V7", "0111");
    h.addOrder("V7", "0106");
    h.addOrder("V7", "3130");
    h.addOrder("V7", "0111");
    h.addVehicle("V8", "0112");
    h.addOrder("V8", "0107");
    h.addOrder("V8", "3015");
    h.addOrder("V8", "0112");
    h.addVehicle("V9", "0113");
    h.addOrder("V9", "0108");
    h.addOrder("V9", "3129");
    h.addOrder("V9", "0113");
    h.addVehicle("V10", "0114");
    h.addOrder("V10", "0109");
    h.addOrder("V10", "3016");
    h.addOrder("V10", "0114");
    boolean drained = h.run(600);
    System.out.println("[faithful] tenAgent drained=" + drained);
    assertThat(drained).as("10-agent cargo loop through the REAL scheduler").isTrue();
  }

  @Test
  void twentyAgentRealScheduler() throws Exception {
    SchedulerFaithfulHarness h = harness();
    String[] starts = {
        "1001", "1002", "1003", "1004", "1005",
        "0110", "0111", "0112", "0113", "0114",
        "0115", "0116", "0117", "0118", "0119",
        "0120", "0121", "0122", "0123", "0124"
    };
    String[] pickups = {
        "0100", "0101", "0102", "0103", "0104",
        "0105", "0106", "0107", "0108", "0109",
        "0125", "0126", "0127", "0128", "0129",
        "0130", "0131", "0132", "0133", "0134"
    };
    String[] deliveries = {
        "3009", "3018", "3027", "3036", "3045",
        "3054", "3063", "3072", "3081", "3090",
        "3099", "3108", "3117", "3126", "3135",
        "3144", "3153", "3162", "3171", "3180"
    };
    for (int i = 0; i < starts.length; i++) {
      String v = "V" + (i + 1);
      h.addVehicle(v, starts[i]);
      h.addOrder(v, pickups[i]);
      h.addOrder(v, deliveries[i]);
    }

    boolean drained = h.run(1500);
    System.out.println("[faithful] twentyAgent drained=" + drained);
    assertThat(drained).as("20-agent start->pickup->deliver through the REAL scheduler").isTrue();
  }

  @Test
  void twentyFiveAgentRealScheduler() throws Exception {
    SchedulerFaithfulHarness h = harness();
    String[] starts = {
        "1001", "1002", "1003", "1004", "1005",
        "0110", "0111", "0112", "0113", "0114",
        "0115", "0116", "0117", "0118", "0119",
        "0120", "0121", "0122", "0123", "0124",
        "0135", "0136", "0137", "0138", "0139"
    };
    String[] pickups = {
        "0100", "0101", "0102", "0103", "0104",
        "0105", "0106", "0107", "0108", "0109",
        "0125", "0126", "0127", "0128", "0129",
        "0130", "0131", "0132", "0133", "0134",
        "0140", "0141", "0142", "0143", "0144"
    };
    String[] deliveries = new String[25];
    for (int i = 0; i < deliveries.length; i++) {
      deliveries[i] = String.format("%04d", 3007 + 7 * i);
    }
    for (int i = 0; i < starts.length; i++) {
      String v = "V" + (i + 1);
      h.addVehicle(v, starts[i]);
      h.addOrder(v, pickups[i]);
      h.addOrder(v, deliveries[i]);
    }

    boolean drained = h.run(1500);
    System.out.println("[faithful] twentyFiveAgent drained=" + drained);
    assertThat(drained).as("25-agent start->pickup->deliver through the REAL scheduler").isTrue();
  }

  @Test
  void thirtyAgentRealScheduler() throws Exception {
    SchedulerFaithfulHarness h = harness();
    String[] starts = {
        "1000", "1001", "1002", "1003", "1004",
        "1005", "1006", "1007", "1008", "1009",
        "0110", "0111", "0112", "0113", "0114",
        "0115", "0116", "0117", "0118", "0119",
        "0120", "0121", "0122", "0123", "0124",
        "0125", "0126", "0127", "0128", "0129"
    };
    String[] pickups = {
        "0100", "0101", "0102", "0103", "0104",
        "0105", "0106", "0107", "0108", "0109",
        "0130", "0131", "0132", "0133", "0134",
        "0135", "0136", "0137", "0138", "0139",
        "0140", "0141", "0142", "0143", "0144",
        "0145", "0146", "0147", "0148", "0149"
    };
    String[] deliveries = new String[30];
    for (int i = 0; i < deliveries.length; i++) {
      deliveries[i] = String.format("%04d", 3005 + 6 * i);
    }
    for (int i = 0; i < starts.length; i++) {
      String v = "V" + (i + 1);
      h.addVehicle(v, starts[i]);
      h.addOrder(v, pickups[i]);
      h.addOrder(v, deliveries[i]);
    }

    boolean drained = h.run(2000);
    System.out.println("[faithful] thirtyAgent drained=" + drained);
    assertThat(drained).as("30-agent start->pickup->deliver through the REAL scheduler").isTrue();
  }

  @Test
  void capturedKernelDeadlock20260709() throws Exception {
    SchedulerFaithfulHarness h = harness();
    // vehicle @captured point (coord) -> goal | blocked waiting on
    h.addVehicle("V1", "0008"); // (210000,0) east -> 3010 | 0007 (V6)
    h.addOrder("V1", "3010");
    h.addVehicle("V2", "3017"); // (220000,-5000) -> loc0116 | 0006 (V9)
    h.addOrder("V2", "0116");
    h.addVehicle("V3", "0086"); // (205000,5000) -> loc0119 | 0087 (V7)
    h.addOrder("V3", "0119");
    h.addVehicle("V4", "0088"); // (215000,5000) -> loc0117 | 0089 (V8)
    h.addOrder("V4", "0117");
    h.addVehicle("V5", "0011"); // (195000,0) moving -> loc0135 | 0086 (V3)
    h.addOrder("V5", "0135");
    h.addVehicle("V6", "0007"); // (215000,0) east -> 3009 | 0006 (V9) <-- HEAD-ON
    h.addOrder("V6", "3009");
    h.addVehicle("V7", "0087"); // (210000,5000) -> loc0118 | 0088 (V4)
    h.addOrder("V7", "0118");
    h.addVehicle("V8", "0089"); // (220000,5000) east -> 3011 | 0006 (V9)
    h.addOrder("V8", "3011");
    h.addVehicle("V9", "0006"); // (220000,0) west -> 3049 | 0007 (V6) <-- HEAD-ON
    h.addOrder("V9", "3049");
    h.addVehicle("V10", "3018"); // (220000,-10000) -> loc0121 | 3017 (V2)
    h.addOrder("V10", "0121");

    boolean drained = h.run(600);
    System.out.println("[faithful] capturedKernelDeadlock20260709 drained=" + drained);
    assertThat(drained)
        .as("live-captured 10-vehicle head-on (0006<->0007) through the REAL scheduler")
        .isTrue();
  }

  @Test
  void sevenMoversShareOneGoalHeldByAWaitingVehicle() throws Exception {
    SchedulerFaithfulHarness h = harness();
    h.addVehicle("HOLDER", "3049");
    h.addOrder("HOLDER", "3049");
    h.addWait("HOLDER", 40);
    h.addOrder("HOLDER", "1005");

    String[] movers = { "0012", "0011", "0010", "0086", "0085", "0084", "0083" };
    for (int i = 0; i < movers.length; i++) {
      h.addVehicle("M" + (i + 1), movers[i]);
      h.addOrder("M" + (i + 1), "3049");
    }

    boolean drained = h.run(900);
    System.out.println("[faithful] sevenMoversShareOneGoal drained=" + drained);
    assertThat(drained)
        .as("seven vehicles given the SAME goal point must still drain once the vehicle"
            + " sitting on it leaves; they take the point in turn")
        .isTrue();
  }

  @Test
  void capturedKernelDeadlock20260803() throws Exception {
    SchedulerFaithfulHarness h = harness();
    h.addVehicle("V15", "3049");
    h.addOrder("V15", "3049");
    h.addWait("V15", 60);
    h.addOrder("V15", "1005");

    String[][] contenders = {
        { "V2", "0086" }, { "V3", "0084" }, { "V5", "0012" }, { "V6", "0011" },
        { "V9", "0085" }, { "V10", "0010" }, { "V13", "0083" }
    };
    for (String[] c : contenders) {
      h.addVehicle(c[0], c[1]);
      h.addOrder(c[0], "3049");
    }

    String[][] background = {
        { "V1", "3021", "0148" }, { "V4", "3059", "0133" }, { "V7", "3060", "0137" },
        { "V8", "3061", "0142" }, { "V11", "3062", "0145" }, { "V12", "3057", "0130" },
        { "V14", "3058", "0131" }
    };
    for (String[] b : background) {
      h.addVehicle(b[0], b[1]);
      h.addOrder(b[0], b[2]);
    }

    boolean drained = h.run(1200);
    System.out.println("[faithful] capturedKernelDeadlock20260803 drained=" + drained);
    assertThat(drained)
        .as("live-captured 15-vehicle freeze of 2026-08-03 15:07 (positions from the 15:07:30"
            + " plan dump, goals from the 15:08:39 fleet line): seven vehicles were given goal"
            + " 3049 while V15 sat on it")
        .isTrue();
  }

  @Test
  void capturedKernelDeadlockWithContinuity() throws Exception {
    SchedulerFaithfulHarness h = harness();
    h.enforceContinuity(true);
    h.addVehicle("V1", "0008");
    h.addOrder("V1", "3010");
    h.addVehicle("V2", "3017");
    h.addOrder("V2", "0116");
    h.addVehicle("V3", "0086");
    h.addOrder("V3", "0119");
    h.addVehicle("V4", "0088");
    h.addOrder("V4", "0117");
    h.addVehicle("V5", "0011");
    h.addOrder("V5", "0135");
    h.addVehicle("V6", "0007");
    h.addOrder("V6", "3009"); // <-- HEAD-ON with V9
    h.addVehicle("V7", "0087");
    h.addOrder("V7", "0118");
    h.addVehicle("V8", "0089");
    h.addOrder("V8", "3011");
    h.addVehicle("V9", "0006");
    h.addOrder("V9", "3049"); // <-- HEAD-ON with V6
    h.addVehicle("V10", "3018");
    h.addOrder("V10", "0121");

    boolean drained = h.run(600);
    System.out.println("[faithful] capturedKernelDeadlockWithContinuity drained=" + drained);
    assertThat(h.rejectedReroutes())
        .as("the continuity constraint must actually bite, else this scenario proves nothing")
        .isGreaterThan(0);
    assertThat(drained)
        .as("continuity-enforced head-on drains once un-commandable vehicles are pinned"
            + " instead of being planned onto detours nobody executes")
        .isTrue();
  }

  @Test
  void capturedLaneStandoff20260805() throws Exception {
    SchedulerFaithfulHarness h = harness();
    h.addVehicle("V07", "3170");
    h.addVehicle("V15", "3167");
    h.addVehicle("V06", "3166");
    h.addVehicle("V12", "3165");
    h.addVehicle("V11", "3164");
    h.addVehicle("V02", "3163");
    h.addVehicle("V05", "3162");
    h.addVehicle("V10", "3161");

    h.addOrder("V07", "3168");
    h.addOrder("V15", "3169");
    h.addOrder("V06", "3168");
    h.addOrder("V12", "3167");
    h.addOrder("V11", "3166");
    h.addOrder("V02", "3165");
    h.addOrder("V05", "3164");
    h.addOrder("V10", "3163");

    boolean drained = h.run(600);
    System.out.println("[faithful] capturedLaneStandoff20260805 drained=" + drained
        + " V07 plannedGoal=" + h.firstPlannedGoalOf("V07")
        + " V15 plannedGoal=" + h.firstPlannedGoalOf("V15"));

    assertThat(h.firstPlannedGoalOf("V07"))
        .as("live capture of 2026-08-05 18:50-19:00 (positions from the 18:58:10 plan dump,"
            + " goals from the 18:58:10 fleet line). V06 and V07 are handed the SAME goal 3168."
            + " V07 sits on the deepest cell 3170 and is retreating outwards, so under"
            + " DeliverySlotEngine's deepest-first fill its order is necessarily the older one"
            + " and it must keep 3168; addOrder call order encodes that. Before claim ranking,"
            + " Collections.sort(agents) handed 3168 to V06 on name order alone and re-aimed"
            + " V07 at surrogate 3169, which then stole V15's real goal in turn")
        .isEqualTo("3168");
    assertThat(h.firstPlannedGoalOf("V15"))
        .as("V15 must keep its own goal: it only lost 3169 because V07's surrogate took it,"
            + " so fixing V07's claim must collapse the whole cascade, not just its first link")
        .isEqualTo("3169");
    assertThat(drained)
        .as("on master there is no footprint constraint, so LaCAM can still turn out of the"
            + " aisle sideways and the fleet drains either way; this asserts the run stays"
            + " healthy, it is NOT what this test is about")
        .isTrue();
  }

  private SchedulerFaithfulHarness laneStandoffFleet() throws Exception {
    SchedulerFaithfulHarness h = harness();
    h.addVehicle("V07", "3170");
    h.addVehicle("V15", "3167");
    h.addVehicle("V06", "3166");
    h.addVehicle("V12", "3165");
    h.addVehicle("V11", "3164");
    h.addVehicle("V02", "3163");
    h.addVehicle("V05", "3162");
    h.addVehicle("V10", "3161");
    return h;
  }

  private void laneStandoffOrders(SchedulerFaithfulHarness h) {
    h.addOrder("V07", "3168");
    h.addOrder("V15", "3169");
    h.addOrder("V06", "3168");
    h.addOrder("V12", "3167");
    h.addOrder("V11", "3166");
    h.addOrder("V02", "3165");
    h.addOrder("V05", "3164");
    h.addOrder("V10", "3163");
  }

}
