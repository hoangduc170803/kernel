package org.opentcs.fms.kernel.mapf.adg;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.entry;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.kernel.mapf.BodyState;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.pibt.Footprint;

class AdgBuilderTest {

  private static final int ALL_COORDINATED = Integer.MAX_VALUE;

  private static final String FOOTPRINT = "fms.pibt.footprint";

  private static final String[] GRID_POINTS = {
      "0101", "0082", "0083", "0084", "0013", "0012", "0011" };
  private static final double[][] GRID_XY = {
      { 1000, 2000 }, { 0, 1000 }, { 1000, 1000 }, { 2000, 1000 },
      { 0, 0 }, { 1000, 0 }, { 2000, 0 } };
  private static final String[][] GRID_LINKS = {
      { "0101", "0083" }, { "0082", "0083" }, { "0083", "0084" },
      { "0013", "0012" }, { "0012", "0011" },
      { "0082", "0013" }, { "0083", "0012" }, { "0084", "0011" } };

  private static final String[] AISLE_POINTS = {
      "0085", "0086", "0009", "0008",
      "0104", "0114", "0124", "0134", "0144", "0154", "0164" };
  private static final double[][] AISLE_XY = {
      { 40000, 1000 }, { 41000, 1000 }, { 41000, 0 }, { 42000, 0 },
      { 41000, 2000 }, { 41000, 3000 }, { 41000, 4000 },
      { 41000, 5000 }, { 41000, 6000 }, { 41000, 7000 }, { 41000, 8000 } };
  private static final String[][] AISLE_LINKS = {
      { "0085", "0086" }, { "0086", "0009" }, { "0009", "0008" },
      { "0104", "0086" }, { "0114", "0104" }, { "0124", "0114" },
      { "0134", "0124" }, { "0144", "0134" }, { "0154", "0144" }, { "0164", "0154" } };

  private static final Set<String> LOADED = Set.of("A", "B");
  private static final Set<String> ALL_LOADED = Set.of("A", "B", "C", "D");
  private static final Set<String> EMPTY = Set.of();

  private Roadmap grid;

  @BeforeEach
  void enableFootprint() {
    System.setProperty(FOOTPRINT, "true");
    grid = roadmapOf(GRID_POINTS, GRID_XY, GRID_LINKS);
  }

  @AfterEach
  void restoreFootprint() {
    System.clearProperty(FOOTPRINT);
  }

  @Test
  void aTurnBesideAStraightRunConstrainsItThoughNoCellIsShared() {
    Map<String, List<String>> plan = Map.of(
        "A", List.of("0013", "0013", "0013", "0012", "0011"),
        "B", List.of("0101", "0083", "0084", "0084", "0084"));

    Adg adg = build(plan, grid, bodies(LOADED));

    assertThat(adg.dependencies())
        .containsExactly(entry(new Step("A", 1), List.of(new Step("B", 1))));
  }

  @Test
  void theSameTurnConstrainsNothingWhenBothVehiclesRunEmpty() {
    Map<String, List<String>> plan = Map.of(
        "A", List.of("0013", "0013", "0013", "0012", "0011"),
        "B", List.of("0101", "0083", "0084", "0084", "0084"));

    Adg adg = build(plan, grid, bodies(EMPTY));

    assertThat(adg.dependencies()).isEmpty();
  }

  @Test
  void aHandoverWaitsForTheOccupantToLeaveTheSharedCellNotThePreviousOne() {
    Map<String, List<String>> plan = Map.of(
        "A", List.of("0013", "0012", "0011", "0011"),
        "B", List.of("0083", "0083", "0083", "0012"));

    Adg adg = build(plan, grid, bodies(LOADED));

    assertThat(adg.dependencies())
        .containsExactly(entry(new Step("B", 1), List.of(new Step("A", 1))));
  }

  @Test
  void parallelRowsOnePitchApartConstrainNothing() {
    Map<String, List<String>> plan = Map.of(
        "A", List.of("0013", "0012", "0011", "0011"),
        "B", List.of("0082", "0082", "0083", "0084"));

    Adg adg = build(plan, grid, bodies(LOADED));

    assertThat(adg.dependencies()).isEmpty();
  }

  @Test
  void routesAreTheCompressedPathsWithoutTheWaits() {
    Map<String, List<String>> plan = Map.of(
        "A", List.of("0013", "0013", "0013", "0012", "0011"),
        "B", List.of("0101", "0083", "0084", "0084", "0084"));

    Adg adg = build(plan, grid, bodies(LOADED));

    assertThat(adg.routeOf("A")).containsExactly("0013", "0012", "0011");
    assertThat(adg.pointOf(new Step("A", 1))).isEqualTo("0012");
  }

  @Test
  void theWaiterIsHeldUntilTheBlockerFinishesTheTurnPastTheSharedCell() {
    Map<String, List<String>> plan = Map.of(
        "A", List.of("0085", "0085", "0085", "0085", "0085", "0085",
            "0085", "0085", "0085", "0085", "0086"),
        "B", List.of("0164", "0154", "0144", "0134", "0124", "0114",
            "0104", "0086", "0009", "0008", "0008"));

    Adg adg = build(
        plan, roadmapOf(AISLE_POINTS, AISLE_XY, AISLE_LINKS), bodies(LOADED));

    assertThat(adg.predecessorsOf(new Step("A", 1)))
        .containsExactly(new Step("B", 8));
    assertThat(adg.dependencies()).hasSize(1);
  }

  @Test
  void theBlockerLeavingTheSharedCellIsNotEnoughToReleaseTheWaiter() {
    Map<String, List<String>> plan = Map.of(
        "A", List.of("0085", "0085", "0085", "0085", "0085", "0085",
            "0085", "0085", "0085", "0085", "0086"),
        "B", List.of("0164", "0154", "0144", "0134", "0124", "0114",
            "0104", "0086", "0009", "0008", "0008"));

    Adg adg = build(
        plan, roadmapOf(AISLE_POINTS, AISLE_XY, AISLE_LINKS), bodies(LOADED));

    assertThat(adg.predecessorsOf(new Step("A", 1)))
        .doesNotContain(new Step("B", 7));
  }

  @Test
  void aTurnTheBlockerHasNotStartedYetConstrainsNobody() {
    Map<String, List<String>> plan = Map.of(
        "A", List.of("0124", "0114", "0104", "0104", "0104", "0104"),
        "B", List.of("0085", "0086", "0086", "0086", "0086", "0009"));

    Adg adg = build(
        plan, roadmapOf(AISLE_POINTS, AISLE_XY, AISLE_LINKS), bodies(LOADED));

    assertThat(adg.dependencies())
        .as("B only pivots on the tick it leaves 0086; while A arrives at 0104 it is resting"
            + " there, and the plan the solver certified has the two coexisting")
        .isEmpty();
  }

  @Test
  void theSameTurnConstrainsTheWaiterWhenTheBlockerReallyPivotsOnArrival() {
    Map<String, List<String>> plan = Map.of(
        "A", List.of("0124", "0114", "0104", "0104", "0104", "0104"),
        "B", List.of("0085", "0086", "0009", "0009", "0009", "0009"));

    Adg adg = build(
        plan, roadmapOf(AISLE_POINTS, AISLE_XY, AISLE_LINKS), bodies(LOADED));

    assertThat(adg.dependencies())
        .containsExactly(entry(new Step("A", 2), List.of(new Step("B", 1))));
  }

  @Test
  void aBlockerParkedOnItsLastStepIsNotMadeIntoAWaitNobodyCanRelease() {
    Map<String, List<String>> plan = Map.of(
        "A", List.of("0085", "0086", "0086", "0086", "0104"),
        "B", List.of("0009", "0009", "0009", "0009", "0009"));

    Adg adg = build(
        plan, roadmapOf(AISLE_POINTS, AISLE_XY, AISLE_LINKS), bodies(LOADED));

    assertThat(adg.predecessorsOf(new Step("A", 1)))
        .as("0009 is the last step of B's route: a precedence on it demands a departure that"
            + " never comes, and holds A until the next replan")
        .isEmpty();
  }

  @Test
  void twoIndependentWaitsOnOneMapEachKeepTheirOwnBlocker() {
    Adg adg = build(gridAndAisleMerged(), gridAndAisleOnOneMap(), bodies(ALL_LOADED));

    assertThat(adg.dependencies()).containsOnly(
        entry(new Step("A", 1), List.of(new Step("B", 8))),
        entry(new Step("C", 1), List.of(new Step("D", 1))));
  }

  @Test
  void neitherWaiterIsHeldByTheOtherClusterThirtyEightMetresAway() {
    Adg adg = build(gridAndAisleMerged(), gridAndAisleOnOneMap(), bodies(ALL_LOADED));

    assertThat(adg.pointOf(new Step("A", 0))).isEqualTo("0085");
    assertThat(adg.pointOf(new Step("A", 1))).isEqualTo("0086");
    assertThat(adg.predecessorsOf(new Step("A", 1)))
        .allMatch(blocker -> blocker.vehicle().equals("B"));

    assertThat(adg.pointOf(new Step("C", 0))).isEqualTo("0013");
    assertThat(adg.pointOf(new Step("C", 1))).isEqualTo("0012");
    assertThat(adg.predecessorsOf(new Step("C", 1)))
        .allMatch(blocker -> blocker.vehicle().equals("D"));
  }

  private static Map<String, List<String>> gridAndAisleMerged() {
    return Map.of(
        "A", List.of("0085", "0085", "0085", "0085", "0085", "0085",
            "0085", "0085", "0085", "0085", "0086"),
        "B", List.of("0164", "0154", "0144", "0134", "0124", "0114",
            "0104", "0086", "0009", "0008", "0008"),
        "C", List.of("0013", "0013", "0013", "0012", "0011", "0011",
            "0011", "0011", "0011", "0011", "0011"),
        "D", List.of("0101", "0083", "0084", "0084", "0084", "0084",
            "0084", "0084", "0084", "0084", "0084"));
  }

  private static final String[] TAIL_POINTS = {
      "P0", "P1", "P2", "X", "Y", "Q0", "Q1", "Q2", "Q3" };
  private static final double[][] TAIL_XY = {
      { 0, 0 }, { 1000, 0 }, { 2000, 0 }, { 3000, 0 }, { 4000, 0 },
      { 0, 2000 }, { 1000, 2000 }, { 2000, 2000 }, { 3000, 2000 } };
  private static final String[][] TAIL_LINKS = {
      { "P0", "P1" }, { "P1", "P2" }, { "P2", "X" }, { "X", "Y" },
      { "Q0", "Q1" }, { "Q1", "Q2" }, { "Q2", "Q3" }, { "Q3", "X" } };

  @Test
  void edgesReachTheTailOfThePlanNotJustItsHead() {
    Map<String, List<String>> plan = Map.of(
        "A", List.of("P0", "P1", "P2", "X", "Y"),
        "B", List.of("Q0", "Q1", "Q2", "Q3", "X"));

    Adg adg = build(plan, roadmapOf(TAIL_POINTS, TAIL_XY, TAIL_LINKS), bodies(LOADED));

    assertThat(adg.dependencies())
        .as("trimming the tail out of the precedence was tried and deadlocked the 20-agent"
            + " fleet: what the solver certifies and what the module enforces must match")
        .containsExactly(entry(new Step("B", 4), List.of(new Step("A", 3))));
  }

  @Test
  void theSharedCellStillOrdersTheTwoVehiclesWhenTheFootprintIsOff() {
    System.setProperty(FOOTPRINT, "false");
    Map<String, List<String>> plan = Map.of(
        "A", List.of("0013", "0012", "0011", "0011"),
        "B", List.of("0083", "0083", "0083", "0012"));

    Adg adg = build(plan, grid, bodies(LOADED));

    assertThat(adg.dependencies())
        .as("turning the footprint off must only drop the geometry edges, never the"
            + " point-occupancy edges that carry the plan's timing")
        .containsExactly(entry(new Step("B", 1), List.of(new Step("A", 1))));
  }

  @Test
  void theSharedCellStillOrdersTheTwoVehiclesWithoutAnyFootprintAtAll() {
    Map<String, List<String>> plan = Map.of(
        "A", List.of("0013", "0012", "0011", "0011"),
        "B", List.of("0083", "0083", "0083", "0012"));

    Adg adg = AdgBuilder.build(plan, ALL_COORDINATED, grid, null, bodies(LOADED));

    assertThat(adg.hasNothingToEnforce()).isFalse();
    assertThat(adg.predecessorsOf(new Step("B", 1))).containsExactly(new Step("A", 1));
  }

  @Test
  void onlyTheGeometryEdgesDisappearWhenTheFootprintIsOff() {
    System.setProperty(FOOTPRINT, "false");
    Map<String, List<String>> plan = Map.of(
        "A", List.of("0013", "0013", "0013", "0012", "0011"),
        "B", List.of("0101", "0083", "0084", "0084", "0084"));

    Adg adg = build(plan, grid, bodies(LOADED));

    assertThat(adg.dependencies()).isEmpty();
    assertThat(adg.routeOf("A")).containsExactly("0013", "0012", "0011");
  }

  @Test
  void theProvenStepIsTheLastOneReachedBeforeThePaddingBegins() {
    Map<String, List<String>> plan = Map.of(
        "A", List.of("0013", "0012", "0012", "0012", "0011"));

    Adg adg = AdgBuilder.build(plan, 3, grid, null, bodies(LOADED));

    assertThat(adg.routeOf("A")).containsExactly("0013", "0012", "0011");
    assertThat(adg.coordinatedStepOf("A")).isEqualTo(1);
  }

  @Test
  void aVehicleStandingStillForTheWholeWindowIsProvenOnItsOnlyStep() {
    Map<String, List<String>> plan = Map.of("A", List.of("0012"));

    Adg adg = AdgBuilder.build(plan, 8, grid, null, bodies(LOADED));

    assertThat(adg.coordinatedStepOf("A")).isEqualTo(0);
  }

  @Test
  void noEdgePromisesADepartureThatOnlyThePaddingWouldMake() {
    Map<String, List<String>> plan = Map.of(
        "A", List.of("0013", "0012", "0011", "0011"),
        "B", List.of("0083", "0083", "0083", "0012"));

    assertThat(AdgBuilder.build(plan, ALL_COORDINATED, grid, null, bodies(LOADED))
        .predecessorsOf(new Step("B", 1)))
        .containsExactly(new Step("A", 1));
    assertThat(AdgBuilder.build(plan, 2, grid, null, bodies(LOADED)).dependencies())
        .isEmpty();
  }

  private static Adg build(
      Map<String, List<String>> plan, Roadmap map, BodyState bodies) {
    return AdgBuilder.build(plan, ALL_COORDINATED, map, new Footprint(map), bodies);
  }

  private static BodyState bodies(Set<String> loaded) {
    return new BodyState(
        Map.of("A", Roadmap.DIR_POS_X, "B", Roadmap.DIR_NEG_Y,
            "C", Roadmap.DIR_POS_X, "D", Roadmap.DIR_NEG_Y),
        loaded);
  }

  private static Roadmap roadmapOf(String[] names, double[][] xy, String[][] links) {
    List<Roadmap.Point> points = new ArrayList<>();
    List<Roadmap.Path> paths = new ArrayList<>();
    addPoints(points, names, xy);
    addPaths(paths, links);
    return new Roadmap(new Roadmap.Model(points, paths));
  }

  private static Roadmap gridAndAisleOnOneMap() {
    List<Roadmap.Point> points = new ArrayList<>();
    List<Roadmap.Path> paths = new ArrayList<>();
    addPoints(points, GRID_POINTS, GRID_XY);
    addPoints(points, AISLE_POINTS, AISLE_XY);
    addPaths(paths, GRID_LINKS);
    addPaths(paths, AISLE_LINKS);
    return new Roadmap(new Roadmap.Model(points, paths));
  }

  private static void addPoints(List<Roadmap.Point> points, String[] names, double[][] xy) {
    for (int i = 0; i < names.length; i++) {
      points.add(new Roadmap.Point(names[i], xy[i][0], xy[i][1], 0));
    }
  }

  private static void addPaths(List<Roadmap.Path> paths, String[][] links) {
    for (String[] link : links) {
      paths.add(new Roadmap.Path(link[0] + "<->" + link[1], link[0], link[1], 1, true));
    }
  }

  // --- the brake tick the kinematic model inserts before a turn ----------------

  @Test
  void theBrakeTickBeforeATurnKeepsTheCellHeldForOneExtraTick() {
    Map<String, List<String>> braking = Map.of(
        "B", List.of("0082", "0083", "0083", "0012"),
        "A", List.of("0101", "0101", "0101", "0083"));

    Adg adg = build(braking, grid, bodies(LOADED));

    assertThat(adg.routeOf("B"))
        .as("the wait is compressed out of the driven route, but it must not be compressed out"
            + " of the timing the edges are built from")
        .containsExactly("0082", "0083", "0012");
    assertThat(adg.predecessorsOf(new Step("A", 1)))
        .as("B stands on 0083 through tick 2 to turn, so A entering at tick 3 is ordered behind"
            + " B's step on that cell")
        .containsExactly(new Step("B", 1));
  }

  @Test
  void withoutTheBrakeTickTheSameCellIsFreeATickEarlier() {
    Map<String, List<String>> straightThrough = Map.of(
        "B", List.of("0082", "0083", "0012", "0012"),
        "A", List.of("0101", "0101", "0083", "0083"));

    Adg withoutBrake = build(straightThrough, grid, bodies(LOADED));

    assertThat(withoutBrake.predecessorsOf(new Step("A", 1)))
        .as("B leaves 0083 at tick 2, so A arriving at tick 2 is a legal handover and the edge"
            + " anchors on the shared cell itself; the braking plan above needs A to wait until"
            + " tick 3 for the same edge")
        .containsExactly(new Step("B", 1));
  }
}
