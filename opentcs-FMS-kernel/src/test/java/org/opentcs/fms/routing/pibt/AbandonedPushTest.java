package org.opentcs.fms.routing.pibt;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.offset;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.DistTable;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.StaticBodies;

class AbandonedPushTest {

  private static final int PITCH = 800;

  private static final int MOVER = 0;
  private static final int ESCAPER = 1;
  private static final int TRAPPED = 2;

  private static final double EXPECTED_MAX_REACH = 2 * Math.hypot(420, 400) + 50;

  @BeforeEach
  void enableFootprint() {
    System.setProperty("fms.pibt.footprint", "true");
  }

  @AfterEach
  void restoreDefaults() {
    System.clearProperty("fms.pibt.footprint");
  }

  private static Roadmap crossroads(boolean trappedHasAWayOut) {
    List<Roadmap.Point> points = new ArrayList<>(List.of(
        new Roadmap.Point("west", -PITCH, 0, 0),
        new Roadmap.Point("centre", 0, 0, 0),
        new Roadmap.Point("north", 0, PITCH, 0),
        new Roadmap.Point("far-north", 0, 2 * PITCH, 0),
        new Roadmap.Point("south", 0, -PITCH, 0)));
    List<Roadmap.Path> paths = new ArrayList<>(List.of(
        new Roadmap.Path("w", "west", "centre", PITCH, true),
        new Roadmap.Path("n", "centre", "north", PITCH, true),
        new Roadmap.Path("nn", "north", "far-north", PITCH, true),
        new Roadmap.Path("s", "south", "centre", PITCH, true)));
    if (trappedHasAWayOut) {
      points.add(new Roadmap.Point("far-south", 0, -2 * PITCH, 0));
      paths.add(new Roadmap.Path("ss", "far-south", "south", PITCH, true));
    }
    return new Roadmap(new Roadmap.Model(points, paths));
  }

  private record Outcome(Roadmap map, int[] next, PibtEngine engine) {

    String where(int agent) {
      return map.nameOf(next[agent]);
    }
  }

  private static Outcome planOneStep(boolean trappedHasAWayOut) {
    Roadmap map = crossroads(trappedHasAWayOut);
    int[] starts = {map.idOf("west"), map.idOf("north"), map.idOf("south")};
    int[] goals = {map.idOf("centre"), map.idOf("north"), map.idOf("south")};
    PibtEngine engine = new PibtEngine(map, 3, 0, DistTable.turnAware(map, goals));
    int[] next = engine.step(
        starts,
        new int[] {MOVER, ESCAPER, TRAPPED},
        List.of(),
        new int[] {Roadmap.DIR_POS_X, Roadmap.DIR_POS_Y, Roadmap.DIR_POS_Y},
        new boolean[] {true, false, false},
        StaticBodies.NONE,
        null);
    return new Outcome(map, next, engine);
  }

  private static boolean restingBodiesClash(
      Roadmap map, String a, int noseA, boolean loadedA, String b, int noseB, boolean loadedB) {
    return new Footprint(map)
        .bodiesClash(map.idOf(a), noseA, loadedA, map.idOf(b), noseB, loadedB);
  }

  @Test
  void theFixtureUsesTheClearanceTheTestSuiteWasSizedFor() {
    assertThat(new Footprint(crossroads(false)).maxReach())
        .as("every distance in this test was picked against a 50mm clearance and the stock"
            + " 840x425 / 800x800 bodies; Footprint reads both into static finals at class-load"
            + " time, so a different -Dfms.pibt.clearanceMm silently invalidates the fixture"
            + " instead of failing it")
        .isCloseTo(EXPECTED_MAX_REACH, offset(0.01));
  }

  @Test
  void theGeometryPutsTheEscaperInTheMoversWayOnlyOnceTheMoverHasArrived() {
    Roadmap map = crossroads(false);

    assertThat(restingBodiesClash(
        map, "centre", Roadmap.DIR_POS_X, true, "north", Roadmap.DIR_POS_Y, false))
        .as("a loaded body at centre reaches 400mm north and an empty body nosed along the"
            + " column reaches 420mm south, so 870mm is needed across a 800mm pitch")
        .isTrue();

    assertThat(restingBodiesClash(
        map, "west", Roadmap.DIR_POS_X, true, "north", Roadmap.DIR_POS_Y, false))
        .as("from west the same two bodies are a pitch apart in x, where only 682.5mm is"
            + " needed, so the escaper has no reason to move unless the mover commits to centre")
        .isFalse();
  }

  @Test
  void aPushThatSucceedsBesideOneThatFailsIsNotRolledBack() {
    Outcome outcome = planOneStep(false);

    assertThat(outcome.next()).as("the step itself resolves, it is the content that is wrong")
        .isNotNull();
    assertThat(outcome.where(MOVER))
        .as("centre is rejected because the trapped agent cannot clear it, so the mover holds")
        .isEqualTo("west");
    assertThat(outcome.where(TRAPPED))
        .as("its only neighbour is centre and centre was taken while the mover was still"
            + " speculating, so it never had a move")
        .isEqualTo("south");

    assertThat(outcome.engine().counters().footprintPushes)
        .as("the mover speculatively pushed both neighbours before giving up on centre")
        .isGreaterThanOrEqualTo(2);

    assertThat(outcome.where(ESCAPER))
        .as("the escaper only ever moved to clear a body that never arrived: the mover"
            + " retracted centre, yet clearsUnplannedMotions undoes to[i]/occNext[u] for the"
            + " mover alone and leaves to[ESCAPER] pointing at far-north")
        .isEqualTo("north");
  }

  @Test
  void theEscaperStandingStillWouldHaveBeenAPerfectlyLegalConfiguration() {
    Outcome outcome = planOneStep(false);

    assertThat(outcome.where(MOVER)).isEqualTo("west");
    assertThat(restingBodiesClash(
        outcome.map(), "west", Roadmap.DIR_POS_X, true, "north", Roadmap.DIR_POS_Y, false))
        .as("nothing in the configuration the solver actually returned required the escaper"
            + " to leave north, which is what makes its step waste rather than yield")
        .isFalse();
  }

  @Test
  void whenEveryPushCanSucceedTheMoverKeepsTheCellAndBothNeighboursYield() {
    Outcome outcome = planOneStep(true);

    assertThat(outcome.next()).isNotNull();
    assertThat(outcome.where(MOVER))
        .as("with an escape hatch behind the trapped agent the whole push chain holds")
        .isEqualTo("centre");
    assertThat(outcome.where(ESCAPER)).isEqualTo("far-north");
    assertThat(outcome.where(TRAPPED)).isEqualTo("far-south");
  }
}
