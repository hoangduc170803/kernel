package org.opentcs.fms.routing.pibt;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.DistTable;
import org.opentcs.fms.routing.Lacam2Solver;
import org.opentcs.fms.routing.MapfInstance;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.SolverParams;
import org.opentcs.fms.routing.StaticBodies;

class FootprintYieldTest {

  private static final String ENABLED = "fms.pibt.footprint";

  private static final double PITCH = 950;

  private static String savedEnabled;

  @BeforeAll
  static void enableFootprint() {
    savedEnabled = System.getProperty(ENABLED);
    System.setProperty(ENABLED, "true");
  }

  @AfterAll
  static void restoreFootprint() {
    if (savedEnabled == null) {
      System.clearProperty(ENABLED);
    }
    else {
      System.setProperty(ENABLED, savedEnabled);
    }
  }

  private static Lacam2Solver.Result solveLoaded(
      Roadmap map, String leaderStart, String leaderGoal, String followerStart,
      String followerGoal, int startNose) {
    return new Lacam2Solver(map).solve(
        new MapfInstance(
            new int[] {map.idOf(leaderStart), map.idOf(followerStart)},
            new int[] {map.idOf(leaderGoal), map.idOf(followerGoal)}),
        5_000, 1, Lacam2Solver.Objective.SUM_OF_LOSS, Lacam2Solver.DEFAULT_RESTART_RATE,
        200_000, SolverParams.WINDOW_FULL, null,
        new int[] {startNose, startNose}, new boolean[] {true, true},
        null, null, StaticBodies.NONE);
  }

  private static String whereIs(Roadmap map, Lacam2Solver.Result plan, int tick, int agent) {
    return map.nameOf(plan.solution().get(tick)[agent]);
  }

  private static int turnsAt(
      Footprint footprint, Lacam2Solver.Result plan, int tick, int startNose) {
    List<int[]> configs = plan.solution();
    int[] axis = {startNose, startNose};
    for (int t = 0; t < tick; t++) {
      for (int a = 0; a < axis.length; a++) {
        axis[a] = footprint.noseAfter(axis[a], configs.get(t)[a], configs.get(t + 1)[a]);
      }
    }
    int turns = 0;
    for (int a = 0; a < axis.length; a++) {
      if (footprint.isRotation(axis[a], configs.get(tick)[a], configs.get(tick + 1)[a])) {
        turns++;
      }
    }
    return turns;
  }

  private static void assertNoStepOverlaps(
      Footprint footprint, Lacam2Solver.Result plan, int startNose, boolean loaded) {
    List<int[]> configs = plan.solution();
    int[] axis = {startNose, startNose};
    for (int t = 0; t + 1 < configs.size(); t++) {
      int[] before = configs.get(t);
      int[] after = configs.get(t + 1);
      assertThat(footprint.collideMoving(
        moving(before[0], after[0], axis[0], loaded),
        moving(before[1], after[1], axis[1], loaded)))
          .describedAs("loaded %s, step %d", loaded, t)
          .isFalse();
      for (int a = 0; a < axis.length; a++) {
        axis[a] = footprint.noseAfter(axis[a], before[a], after[a]);
      }
    }
  }

  private static void assertConstraintsCannotForceBothTurns(
      Roadmap map, String leaderPivot, String leaderGoal, String followerPivot,
      String followerGoal, int startNose) {
    int[] config = {map.idOf(leaderPivot), map.idOf(followerPivot)};
    DistTable dist = DistTable.weighted(
        map, new int[] {map.idOf(leaderGoal), map.idOf(followerGoal)});

    int[] next = new PibtEngine(map, 2, 7, dist).step(
        config, new int[] {0, 1},
        List.of(
            new PibtEngine.Constraint(0, map.idOf(leaderGoal)),
            new PibtEngine.Constraint(1, map.idOf(followerGoal))),
        new int[] {startNose, startNose}, new boolean[] {true, true},
        StaticBodies.NONE, null);

    assertThat(next)
        .describedAs("constraints skip the per-candidate gates, so the configuration itself"
            + " has to be rejected")
        .isNull();
  }

  @Nested
  class LeavingThePickupRow {

    private final Roadmap map = pickupRow();
    private final Footprint footprint = new Footprint(map);

    private Roadmap pickupRow() {
      List<Roadmap.Point> points = List.of(
          new Roadmap.Point("0081", 0, 0, 0),
          new Roadmap.Point("0082", PITCH, 0, 0),
          new Roadmap.Point("0083", 2 * PITCH, 0, 0),
          new Roadmap.Point("0084", 3 * PITCH, 0, 0),
          new Roadmap.Point("0100", PITCH, PITCH, 0),
          new Roadmap.Point("0101", 2 * PITCH, PITCH, 0));
      List<Roadmap.Path> paths = List.of(
          new Roadmap.Path("0081 --- 0082", "0081", "0082", PITCH, true),
          new Roadmap.Path("0082 --- 0083", "0082", "0083", PITCH, true),
          new Roadmap.Path("0083 --- 0084", "0083", "0084", PITCH, true),
          new Roadmap.Path("0100 --- 0082", "0100", "0082", PITCH, true),
          new Roadmap.Path("0101 --- 0083", "0101", "0083", PITCH, true));
      return new Roadmap(new Roadmap.Model(points, paths));
    }

    private Lacam2Solver.Result plan() {
      return solveLoaded(map, "0100", "0081", "0101", "0084", Roadmap.DIR_POS_Y);
    }

    @Test
    void theFollowerHoldsInsideTheLaneInsteadOfEnteringTheCorridor() {
      Lacam2Solver.Result plan = plan();

      assertThat(plan.solved()).isTrue();
      assertThat(whereIs(map, plan, 1, 0)).isEqualTo("0082");
      assertThat(whereIs(map, plan, 1, 1))
          .describedAs("a loaded body sweeps 565.7mm turning and stands 400mm wide, so the"
              + " follower may not even enter the corridor beside the pivot")
          .isEqualTo("0101");
      assertThat(whereIs(map, plan, 2, 1)).isEqualTo("0101");
    }

    @Test
    void theLeaderTurnsFirstAndTheFollowerOnlyAfterThePivotIsClear() {
      Lacam2Solver.Result plan = plan();

      assertThat(turnsAt(footprint, plan, 1, Roadmap.DIR_POS_Y)).isEqualTo(1);
      assertThat(whereIs(map, plan, 2, 0)).isEqualTo("0081");
      assertThat(whereIs(map, plan, 3, 1)).isEqualTo("0083");
      assertThat(whereIs(map, plan, 4, 1)).isEqualTo("0084");
    }

    @Test
    void noSolutionStepEverPlacesTwoBodiesOnTopOfEachOther() {
      assertNoStepOverlaps(footprint, plan(), Roadmap.DIR_POS_Y, true);
    }

    @Test
    void aLowLevelConstraintCannotForceTwoLoadedVehiclesToTurnAbreast() {
      assertConstraintsCannotForceBothTurns(map, "0082", "0081", "0083", "0084",
          Roadmap.DIR_POS_Y);
    }
  }

  @Nested
  class TurningIntoAdjacentBays {

    private final Roadmap map = corridorWithTwoBays();
    private final Footprint footprint = new Footprint(map);

    private Roadmap corridorWithTwoBays() {
      List<Roadmap.Point> points = List.of(
          new Roadmap.Point("0082", 0, 0, 0),
          new Roadmap.Point("0083", PITCH, 0, 0),
          new Roadmap.Point("0084", 2 * PITCH, 0, 0),
          new Roadmap.Point("0085", 3 * PITCH, 0, 0),
          new Roadmap.Point("0012", PITCH, -PITCH, 0),
          new Roadmap.Point("0011", 2 * PITCH, -PITCH, 0));
      List<Roadmap.Path> paths = List.of(
          new Roadmap.Path("0082 --- 0083", "0082", "0083", PITCH, true),
          new Roadmap.Path("0083 --- 0084", "0083", "0084", PITCH, true),
          new Roadmap.Path("0084 --- 0085", "0084", "0085", PITCH, true),
          new Roadmap.Path("0083 --- 0012", "0083", "0012", PITCH, true),
          new Roadmap.Path("0084 --- 0011", "0084", "0011", PITCH, true));
      return new Roadmap(new Roadmap.Model(points, paths));
    }

    private Lacam2Solver.Result plan() {
      return solveLoaded(map, "0082", "0012", "0085", "0011", Roadmap.DIR_POS_X);
    }

    @Test
    void thePivotsAreOnePitchApartSoTwoLoadedTurnsWouldOverlap() {
      assertThat(footprint.collideMoving(
        moving(map.idOf("0083"), map.idOf("0012"), Roadmap.DIR_POS_X, true),
        moving(map.idOf("0084"), map.idOf("0011"), Roadmap.DIR_POS_X, true)))
          .describedAs("two loaded swept discs need 2 x 565.7mm, the pivots are %.0fmm apart",
              PITCH)
          .isTrue();
    }

    @Test
    void aLoadedVehicleMayNotEvenRestOnThePivotBesideATurningOne() {
      assertThat(footprint.collideMoving(
        moving(map.idOf("0084"), map.idOf("0011"), Roadmap.DIR_POS_X, true),
        resting(map.idOf("0083"), Roadmap.DIR_POS_X, true)))
          .describedAs("565.7mm of swept disc plus 400mm of resting cargo exceeds %.0fmm", PITCH)
          .isTrue();
    }

    @Test
    void theLeaderHoldsShortOfItsPivotUntilTheFollowerHasClearedTheCorridor() {
      Lacam2Solver.Result plan = plan();

      assertThat(plan.solved()).isTrue();
      assertThat(whereIs(map, plan, 1, 0)).isEqualTo("0082");
      assertThat(whereIs(map, plan, 2, 0))
          .describedAs("stepping onto 0083 would block the follower's pivot")
          .isEqualTo("0082");
      assertThat(whereIs(map, plan, 2, 1)).isEqualTo("0011");
    }

    @Test
    void theLeaderPivotsOnlyAfterTheFollowerIsParkedInItsBay() {
      Lacam2Solver.Result plan = plan();

      assertThat(whereIs(map, plan, 3, 0)).isEqualTo("0083");
      assertThat(whereIs(map, plan, 4, 0)).isEqualTo("0012");
      assertThat(whereIs(map, plan, 4, 1)).isEqualTo("0011");
    }

    @Test
    void atMostOneVehiclePivotsInAnyStep() {
      Lacam2Solver.Result plan = plan();

      for (int t = 0; t + 1 < plan.solution().size(); t++) {
        assertThat(turnsAt(footprint, plan, t, Roadmap.DIR_POS_X))
            .describedAs("step %d", t)
            .isLessThanOrEqualTo(1);
      }
    }

    @Test
    void noSolutionStepEverPlacesTwoBodiesOnTopOfEachOther() {
      assertNoStepOverlaps(footprint, plan(), Roadmap.DIR_POS_X, true);
    }

    @Test
    void aLowLevelConstraintCannotForceBothPivotsInTheSameStep() {
      assertConstraintsCannotForceBothTurns(map, "0083", "0012", "0084", "0011",
          Roadmap.DIR_POS_X);
    }
  }

  private static Footprint.Motion moving(int from, int to, int nose, boolean loaded) {
    return new Footprint.Motion(from, to, nose, loaded);
  }

  private static Footprint.Motion resting(int at, int nose, boolean loaded) {
    return Footprint.Motion.resting(at, nose, loaded);
  }
}
