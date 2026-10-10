package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.pibt.Footprint;

class AdjacentPivotFootprintTest {

  private static final String MAP_FILE = "v7-vda5050.xml";

  private static final String A_START = "0083";
  private static final String A_PIVOT = "0084";
  private static final String A_GOAL = "0102";
  private static final String B_START = "0010";
  private static final String B_PIVOT = "0011";
  private static final String B_GOAL = "3057";

  private static final int A = 0;
  private static final int B = 1;

  private static Roadmap map;

  @BeforeAll
  static void loadTheProductionMap() throws Exception {
    File file = RoadmapXmlLoader.findMap(MAP_FILE);
    assumeTrue(file != null, MAP_FILE + " not found; skipping");
    map = new Roadmap(RoadmapXmlLoader.load(file));
  }

  @AfterEach
  void restoreDefaults() {
    System.clearProperty("fms.pibt.footprint");
  }

  private static Footprint.Motion pivotOf(String at, String towards, int noseBefore, boolean load) {
    return new Footprint.Motion(map.idOf(at), map.idOf(towards), noseBefore, load);
  }

  @Test
  void theTwoPivotCellsSitOneMetreApartOnTheSameColumn() {
    for (String point : List.of(A_START, A_PIVOT, A_GOAL, B_START, B_PIVOT, B_GOAL)) {
      assertThat(map.has(point)).as("%s missing from %s", point, MAP_FILE).isTrue();
    }
    assertThat(map.xOf(map.idOf(A_PIVOT))).isEqualTo(map.xOf(map.idOf(B_PIVOT)));
    assertThat(Math.abs(map.yOf(map.idOf(A_PIVOT)) - map.yOf(map.idOf(B_PIVOT)))).isEqualTo(1000.0);
    assertThat(map.neighborsOf(map.idOf(A_PIVOT))).contains(map.idOf(A_GOAL));
    assertThat(map.neighborsOf(map.idOf(B_PIVOT))).contains(map.idOf(B_GOAL));
  }

  @Test
  void twoPivotsOneMetreApartClashOnceEitherVehicleCarriesALoad() {
    assertThat(new Footprint(map).collideMoving(
        pivotOf(A_PIVOT, A_GOAL, Roadmap.DIR_POS_X, true),
        pivotOf(B_PIVOT, B_GOAL, Roadmap.DIR_NEG_X, false)))
        .as("a loaded disc of 580 beside an empty one of 470.7 needs 1100.7mm with the 50mm"
            + " clearance, and the cells are 1000mm apart")
        .isTrue();
  }

  @Test
  void theSameTwoPivotsClearEachOtherWhileBothVehiclesAreEmpty() {
    assertThat(new Footprint(map).collideMoving(
        pivotOf(A_PIVOT, A_GOAL, Roadmap.DIR_POS_X, false),
        pivotOf(B_PIVOT, B_GOAL, Roadmap.DIR_NEG_X, false)))
        .as("two empty discs need 991.4mm and clear 1000mm by 8.6mm, so tightening the loaded"
            + " case must not sweep this one up with it")
        .isFalse();
  }

  @Test
  void noStepOfTheEmittedPlanPutsTheTwoBodiesThroughEachOther() {
    System.setProperty("fms.pibt.footprint", "true");
    Lacam2Solver.Result result = new Lacam2Solver(map).solve(
        new MapfInstance(
            new int[] {map.idOf(A_START), map.idOf(B_START)},
            new int[] {map.idOf(A_GOAL), map.idOf(B_GOAL)}),
        1_000L, 0, Lacam2Solver.Objective.NONE, Lacam2Solver.DEFAULT_RESTART_RATE,
        Lacam2Solver.LOOP_LIMIT_UNBOUNDED, SolverParams.DEFAULT_WINDOW,
        new AgentPriority[] {AgentPriority.NEWCOMER, AgentPriority.NEWCOMER},
        new int[] {Roadmap.DIR_POS_X, Roadmap.DIR_NEG_X},
        new boolean[] {true, false},
        null, null, StaticBodies.NONE);

    assertThat(result.solved()).isTrue();

    Footprint footprint = new Footprint(map);
    List<int[]> plan = result.solution();
    int[] nose = {Roadmap.DIR_POS_X, Roadmap.DIR_NEG_X};
    for (int t = 0; t + 1 < plan.size(); t++) {
      assertThat(footprint.collideMoving(
          new Footprint.Motion(plan.get(t)[A], plan.get(t + 1)[A], nose[A], true),
          new Footprint.Motion(plan.get(t)[B], plan.get(t + 1)[B], nose[B], false)))
          .as("step %d of the emitted plan puts two bodies through each other", t)
          .isFalse();
      nose[A] = footprint.noseAfter(nose[A], plan.get(t)[A], plan.get(t + 1)[A]);
      nose[B] = footprint.noseAfter(nose[B], plan.get(t)[B], plan.get(t + 1)[B]);
    }
  }
}
