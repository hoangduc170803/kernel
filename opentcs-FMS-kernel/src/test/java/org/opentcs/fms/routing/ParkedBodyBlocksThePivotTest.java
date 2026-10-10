package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.pibt.Footprint;

class ParkedBodyBlocksThePivotTest {

  private static final String MAP_FILE = "v7-Xuonghientai_them.xml";

  private static final String START = "0947";
  private static final String JUNCTION = "0944";
  private static final String DIVE = "0055";
  private static final String DETOUR = "0943";
  private static final String REJOIN = "0056";
  private static final String GOAL = "0058";
  private static final String PARKED = "0054";

  private static final int MOVER = 0;
  private static final int WINDOW = 10;
  private static final int SEED = 0;

  private static Roadmap map;

  @BeforeAll
  static void loadTheProductionMap() throws Exception {
    File file = RoadmapXmlLoader.findMap(MAP_FILE);
    assumeTrue(file != null, MAP_FILE + " not found; skipping");
    map = new Roadmap(RoadmapXmlLoader.load(file));
  }

  @BeforeEach
  void armTheFootprint() {
    System.setProperty("fms.pibt.footprint", "true");
  }

  @AfterEach
  void restoreDefaults() {
    System.clearProperty("fms.pibt.footprint");
  }

  private static int id(String point) {
    return map.idOf(point);
  }

  private static Footprint.Motion parkedVehicle4() {
    return Footprint.Motion.resting(id(PARKED), Roadmap.DIR_NEG_Y, false);
  }

  @Test
  void theParkedCellSitsEightHundredMillimetresNorthOfTheDiveCell() {
    for (String point : List.of(START, JUNCTION, DIVE, DETOUR, REJOIN, GOAL, PARKED)) {
      assertThat(map.has(point)).as("%s missing from %s", point, MAP_FILE).isTrue();
    }
    assertThat(map.xOf(id(PARKED))).isEqualTo(map.xOf(id(DIVE)));
    assertThat(Math.abs(map.yOf(id(PARKED)) - map.yOf(id(DIVE))))
        .as("the column pitch that makes the pivot tight")
        .isEqualTo(800.0);
    assertThat(map.neighborsOf(id(JUNCTION))).contains(id(DIVE), id(DETOUR));
    assertThat(map.neighborsOf(id(DIVE))).contains(id(REJOIN));
    assertThat(map.neighborsOf(id(DETOUR))).contains(id(REJOIN));
  }

  @Test
  void avoidingTheParkedBodyCostsOneExtraQuarterTurnAndNothingElse() {
    assertThat(quarterTurnsAlong(JUNCTION, DIVE, REJOIN)).isEqualTo(1);
    assertThat(quarterTurnsAlong(JUNCTION, DETOUR, REJOIN)).isEqualTo(2);

    int detourPremium = Roadmap.QUARTER_TURN_COST;
    float chargeForABodyInTheWay = 2.0f * Roadmap.MAX_STEP_WORK;
    assertThat(chargeForABodyInTheWay)
        .as("both ways are two steps, so the detour only costs one more quarter turn (%d);"
            + " fms.guide.footprintCost would charge %s for the body, which dwarfs it",
            detourPremium, chargeForABodyInTheWay)
        .isGreaterThan(detourPremium);
  }

  private static int quarterTurnsAlong(String a, String b, String c) {
    int nose = Roadmap.DIR_POS_X;
    int turns = 0;
    for (String[] step : new String[][] {{a, b}, {b, c}}) {
      int after = map.noseAfter(nose, id(step[0]), id(step[1]));
      if (Roadmap.axisOfDirection(nose) != Roadmap.axisOfDirection(after)) {
        turns++;
      }
      nose = after;
    }
    return turns;
  }

  @Test
  void standingStillOnTheDiveCellClearsTheParkedBody() {
    assertThat(new Footprint(map).collideMoving(
        new Footprint.Motion(id(JUNCTION), id(DIVE), Roadmap.DIR_POS_X, false),
        parkedVehicle4()))
        .as("half-extents of 212.5 and 420 span 632.5mm of the 800mm pitch, so driving in is"
            + " legal at any clearance the fleet actually runs")
        .isFalse();
  }

  @Test
  void pivotingOnTheDiveCellSweepsThroughTheParkedBody() {
    assertThat(new Footprint(map).collideMoving(
        new Footprint.Motion(id(DIVE), id(REJOIN), Roadmap.DIR_POS_X, false),
        parkedVehicle4()))
        .as("the 470.7mm swept disc plus the 420mm resting half-extent needs 890.7mm and the"
            + " cells are 800mm apart, so the pivot clashes before any clearance is added")
        .isTrue();
  }

  @Test
  void pivotingOnTheDetourCellClearsTheParkedBody() {
    assertThat(new Footprint(map).collideMoving(
        new Footprint.Motion(id(DETOUR), id(REJOIN), Roadmap.DIR_NEG_Y, false),
        parkedVehicle4()))
        .as("the detour pivot is a whole column away from the parked body")
        .isFalse();
  }

  @Test
  void theGuideSteersRoundTheParkedBodyInsteadOfAdvisingThePivotItCannotExecute() {
    int[] advice = solveWith(justVehicle4()).guidePaths()[MOVER];

    assertThat(namesOf(advice))
        .as("the guide never sees the parked body, so it advises the pivot at %s that PIBT"
            + " then refuses with footprintVsPlanned staticBody", DIVE)
        .doesNotContain(DIVE);
  }

  @Test
  void theEmittedPlanNeverEntersTheCellItWouldHaveToBackOutOf() {
    List<String> route = routeOf(solveWith(theWholeParkedFleet()));

    assertThat(route)
        .as("a plan that never leaves the start would pass the next assertion vacuously")
        .endsWith(GOAL);
    assertThat(route)
        .as("entering %s commits the vehicle to reversing out of it", DIVE)
        .doesNotContain(DIVE);
  }

  private static List<String> routeOf(Lacam2Solver.Result result) {
    List<String> visited = new ArrayList<>();
    for (int[] configuration : result.solution()) {
      visited.add(map.nameOf(configuration[MOVER]));
    }
    return visited;
  }

  private static StaticBodies justVehicle4() {
    return new StaticBodies(
        new int[] {id(PARKED)},
        new int[] {Roadmap.DIR_NEG_Y},
        new boolean[] {false});
  }

  private static StaticBodies theWholeParkedFleet() {
    String[] cells = {"0354", "0053", "0376", PARKED, "0378", "0377", "0946"};
    int[] ids = new int[cells.length];
    int[] noses = new int[cells.length];
    for (int body = 0; body < cells.length; body++) {
      ids[body] = id(cells[body]);
      noses[body] = "0946".equals(cells[body]) ? Roadmap.DIR_POS_X : Roadmap.DIR_NEG_Y;
    }
    return new StaticBodies(ids, noses, new boolean[cells.length]);
  }

  private static Lacam2Solver.Result solveWith(StaticBodies parked) {
    Lacam2Solver.Result result = new Lacam2Solver(map).solve(
        new MapfInstance(new int[] {id(START)}, new int[] {id(GOAL)}),
        1_000L, SEED, Lacam2Solver.Objective.NONE, Lacam2Solver.DEFAULT_RESTART_RATE,
        Lacam2Solver.LOOP_LIMIT_UNBOUNDED, WINDOW,
        new AgentPriority[] {AgentPriority.NEWCOMER},
        new int[] {Roadmap.DIR_POS_X},
        new boolean[] {false},
        null, null, parked);
    assertThat(result.solved()).isTrue();
    return result;
  }

  private static List<String> namesOf(int[] path) {
    List<String> names = new ArrayList<>();
    for (int vertex : path) {
      if (vertex >= 0) {
        names.add(map.nameOf(vertex));
      }
    }
    return names;
  }
}
