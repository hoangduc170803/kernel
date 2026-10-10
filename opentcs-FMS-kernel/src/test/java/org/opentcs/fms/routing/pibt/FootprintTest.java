package org.opentcs.fms.routing.pibt;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.Roadmap;

class FootprintTest {

  private static final int COLUMNS = 4;
  private static final int ROWS = 4;

  private static Roadmap gridAt(double pitch) {
    List<Roadmap.Point> points = new ArrayList<>();
    for (int row = 0; row < ROWS; row++) {
      for (int col = 0; col < COLUMNS; col++) {
        points.add(new Roadmap.Point(name(col, row), col * pitch, row * pitch, 0.0));
      }
    }
    List<Roadmap.Path> paths = new ArrayList<>();
    for (int row = 0; row < ROWS; row++) {
      for (int col = 0; col < COLUMNS; col++) {
        if (col + 1 < COLUMNS) {
          paths.add(new Roadmap.Path(
              name(col, row) + "e", name(col, row), name(col + 1, row), pitch, true));
        }
        if (row + 1 < ROWS) {
          paths.add(new Roadmap.Path(
              name(col, row) + "n", name(col, row), name(col, row + 1), pitch, true));
        }
      }
    }
    return new Roadmap(new Roadmap.Model(points, paths));
  }

  private static String name(int col, int row) {
    return col + "_" + row;
  }

  private static Footprint.Motion moving(int from, int to, int nose, boolean loaded) {
    return new Footprint.Motion(from, to, nose, loaded);
  }

  private static Footprint.Motion resting(int at, int nose, boolean loaded) {
    return Footprint.Motion.resting(at, nose, loaded);
  }

  private static int id(Roadmap map, int col, int row) {
    return map.idOf(name(col, row));
  }

  @Test
  void aNeighbourThatParksBesideALoadedBodyIsStillSweptWhenThatBodyTurnsInPlace() {
    Roadmap map = gridAt(1000);
    Footprint footprint = new Footprint(map);
    int pivot = id(map, 1, 1);
    int neighbour = id(map, 2, 1);

    assertThat(footprint.cannotCoexistWith(pivot))
        .as("two loaded bodies park side by side on a 1000 pitch with 420+420+50 of body and"
            + " clearance, so the resting relation leaves the neighbour out")
        .doesNotContain(neighbour);

    assertThat(footprint.cannotCoexistWhileTurningAt(pivot))
        .as("turning in place sweeps hypot(420,400)=580, and 580+420+50=1050 overruns the 1000"
            + " pitch, so no heading saves the neighbour once the pivot starts")
        .contains(neighbour);
  }

  @Test
  void aGridPitchedUnderOneBodyLengthCannotHoldVehiclesOnNeighbouringCells() {
    Roadmap map = gridAt(750);

    assertThat(new Footprint(map).cannotCoexistWith(id(map, 1, 1)))
        .as("bodies are 840 long and 800 across, so on a 750 pitch no heading lets two of them"
            + " sit on adjacent cells; this is the table that tells precedence which cells fall"
            + " out of use while somebody stands next to them")
        .contains(id(map, 1, 0), id(map, 1, 2), id(map, 0, 1), id(map, 2, 1));
  }

  @Test
  void aGridWideEnoughForTwoBodiesSideBySideRulesOutNothing() {
    Roadmap map = gridAt(950);

    assertThat(new Footprint(map).cannotCoexistWith(id(map, 1, 1)))
        .as("950 clears the widest pair of half-extents, so a mainline at this pitch adds no"
            + " gates at all and precedence keeps behaving exactly as it does today")
        .isEmpty();
  }

  @Test
  void theSweptDiscEnclosesTheWholeLoadedEnvelopeNotJustTheCargo() {
    Footprint footprint = new Footprint(gridAt(950));
    assertThat(footprint.sweptRadius(false)).isCloseTo(470.70, within(0.05));
    assertThat(footprint.sweptRadius(true))
        .as("a loaded vehicle is as long as the 840mm body and as wide as the 800mm cargo, so the"
            + " disc that encloses it has radius hypot(420, 400); measuring the 800x800 cargo on"
            + " its own returns 565.7 and leaves the disc 14.3mm short of the real envelope")
        .isCloseTo(580.00, within(0.05));
  }

  @Test
  void twoLoadedVehiclesDiagonallyApartOnASquare750GridOverlap() {
    Roadmap map = gridAt(750);
    Footprint footprint = new Footprint(map);

    assertThat(footprint.collideMoving(
        moving(id(map, 1, 1), id(map, 1, 1), Roadmap.DIR_POS_Y, true),
        resting(id(map, 2, 2), Roadmap.DIR_POS_Y, true)))
        .as("both bodies are 800 across and 840 along while the cells are only 750 apart on each"
            + " axis, so the rectangles overlap on both; projecting onto a single connecting axis"
            + " has no axis to pick on a diagonal and compares 840 against the 1061mm hypotenuse,"
            + " which reports two overlapping vehicles as clear")
        .isTrue();
  }

  @Test
  void twoLoadedVehiclesDiagonallyApartOnASquare950GridDoNotOverlap() {
    Roadmap map = gridAt(950);
    Footprint footprint = new Footprint(map);

    assertThat(footprint.collideMoving(
        moving(id(map, 1, 1), id(map, 1, 1), Roadmap.DIR_POS_Y, true),
        resting(id(map, 2, 2), Roadmap.DIR_POS_Y, true)))
        .as("950 clears the 800mm they need across, so widening the diagonal test must not start"
            + " blocking the aisle pitch the real map is built on")
        .isFalse();
  }

  @Test
  void twoAdjacentEmptyTurnsNoLongerClearEachOtherOnceClearanceIsCharged() {
    Roadmap map = gridAt(950);
    Footprint footprint = new Footprint(map);
    assertThat(footprint.collideMoving(
        moving(id(map, 1, 1), id(map, 1, 2), Roadmap.DIR_POS_X, false),
        moving(id(map, 2, 1), id(map, 2, 2), Roadmap.DIR_POS_X, false)))
        .describedAs("two empty discs of 470.7mm meet across a 950mm pitch with 8.6mm to spare,"
            + " which the shipped 50mm clearance consumes; this pair is the one that decides"
            + " whether clearance is charged at all, so it flips the moment that default moves")
        .isTrue();
  }

  @Test
  void twoAdjacentTurnsCollideWheneverEitherVehicleCarriesALoad() {
    Roadmap map = gridAt(950);
    Footprint footprint = new Footprint(map);
    assertThat(footprint.collideMoving(
        moving(id(map, 1, 1), id(map, 1, 2), Roadmap.DIR_POS_X, true),
        moving(id(map, 2, 1), id(map, 2, 2), Roadmap.DIR_POS_X, false)))
        .describedAs("565.7mm of loaded disc plus 470.7mm of empty disc exceeds 950mm")
        .isTrue();
  }

  @Test
  void theSameEmptyPairClearsOnceThePitchGrowsToOneMetre() {
    Roadmap map = gridAt(1000);
    Footprint footprint = new Footprint(map);
    assertThat(footprint.collideMoving(
        moving(id(map, 1, 1), id(map, 1, 2), Roadmap.DIR_POS_X, false),
        moving(id(map, 2, 1), id(map, 2, 2), Roadmap.DIR_POS_X, false)))
        .isFalse();
  }

  @Test
  void loadedTurnCollidesWithARestingNeighbourShowingItsLongSide() {
    Roadmap map = gridAt(950);
    Footprint footprint = new Footprint(map);
    assertThat(footprint.collideMoving(
        moving(id(map, 1, 1), id(map, 1, 2), Roadmap.DIR_POS_X, true),
        resting(id(map, 2, 1), Roadmap.DIR_POS_X, false)))
        .isTrue();
  }

  @Test
  void loadedTurnClearsARestingNeighbourShowingItsNarrowSide() {
    Roadmap map = gridAt(950);
    Footprint footprint = new Footprint(map);
    assertThat(footprint.collideMoving(
        moving(id(map, 1, 1), id(map, 1, 2), Roadmap.DIR_POS_X, true),
        resting(id(map, 2, 1), Roadmap.DIR_POS_Y, false)))
        .isFalse();
  }

  @Test
  void followingIntoTheCellBeingVacatedIsNotACollision() {
    Roadmap map = gridAt(950);
    Footprint footprint = new Footprint(map);
    assertThat(footprint.collideMoving(
        moving(id(map, 1, 1), id(map, 1, 2), Roadmap.DIR_POS_X, false),
        moving(id(map, 0, 1), id(map, 1, 1), Roadmap.DIR_POS_X, false)))
        .describedAs("a grid step carries no internal time; ordering is the allocator's job")
        .isFalse();
  }

  @Test
  void followingIsStillRejectedWhenTheFollowerAlsoSitsTooCloseWhileTurning() {
    Roadmap map = gridAt(950);
    Footprint footprint = new Footprint(map);
    assertThat(footprint.collideMoving(
        moving(id(map, 1, 1), id(map, 1, 2), Roadmap.DIR_POS_X, true),
        moving(id(map, 0, 1), id(map, 1, 1), Roadmap.DIR_POS_X, false)))
        .describedAs("the follower still stands one cell away while the loaded body sweeps")
        .isTrue();
  }

  @Test
  void diagonalNeighboursNeverCollideEvenWhenBothTurnLoaded() {
    Roadmap map = gridAt(950);
    Footprint footprint = new Footprint(map);
    assertThat(footprint.collideMoving(
        moving(id(map, 1, 1), id(map, 1, 0), Roadmap.DIR_POS_X, true),
        moving(id(map, 2, 2), id(map, 2, 3), Roadmap.DIR_POS_X, true)))
        .describedAs("both stay diagonal throughout; neither ever enters the other's ring")
        .isFalse();
  }

  @Test
  void parallelTravelInAdjacentLanesNeverCollides() {
    Roadmap map = gridAt(950);
    Footprint footprint = new Footprint(map);
    assertThat(footprint.collideMoving(
        moving(id(map, 0, 0), id(map, 1, 0), Roadmap.DIR_POS_X, true),
        moving(id(map, 0, 1), id(map, 1, 1), Roadmap.DIR_POS_X, true)))
        .isFalse();
  }

  private static Roadmap laneWithAReverseEdge(double pitch) {
    List<Roadmap.Point> points = List.of(
        new Roadmap.Point("west", 0.0, 0.0, 0.0),
        new Roadmap.Point("east", pitch, 0.0, 0.0));
    List<Roadmap.Path> paths = List.of(
        new Roadmap.Path("out", "west", "east", pitch, false, false),
        new Roadmap.Path("out~rev", "east", "west", pitch, false, true));
    return new Roadmap(new Roadmap.Model(points, paths));
  }

  @Test
  void drivingAgainstTheNoseIsAnAboutFaceButBackingUpIsNot() {
    Roadmap grid = gridAt(950);
    Footprint onGrid = new Footprint(grid);
    assertThat(onGrid.isRotation(Roadmap.DIR_POS_X, id(grid, 1, 1), id(grid, 0, 1)))
        .as("the vehicle reverses along its own axis instead of spinning, so a nose flip on one"
            + " axis sweeps nothing the two resting footprints do not already cover; no"
            + " production map emits this pair anyway, every reversible edge there is ~rev")
        .isFalse();
    assertThat(onGrid.noseAfter(Roadmap.DIR_POS_X, id(grid, 1, 1), id(grid, 0, 1)))
        .isEqualTo(Roadmap.DIR_NEG_X);
    assertThat(Roadmap.quarterTurnsBetween(Roadmap.DIR_POS_X, Roadmap.DIR_NEG_X))
        .as("the price of the about-face is deliberately kept even though the body does not"
            + " sweep: cost discourages reversing, geometry describes what actually moves")
        .isEqualTo(2);
    assertThat(onGrid.isRotation(Roadmap.DIR_POS_X, id(grid, 1, 1), id(grid, 1, 2))).isTrue();

    Roadmap lane = laneWithAReverseEdge(950);
    Footprint onLane = new Footprint(lane);
    assertThat(onLane.isRotation(Roadmap.DIR_POS_X, lane.idOf("east"), lane.idOf("west")))
        .as("a ~rev edge is the vehicle backing up, which leaves the nose where it was")
        .isFalse();
    assertThat(onLane.noseAfter(Roadmap.DIR_POS_X, lane.idOf("east"), lane.idOf("west")))
        .isEqualTo(Roadmap.DIR_POS_X);
  }

  @Test
  void anUnknownAxisIsTreatedAsTheWidestPossibleBody() {
    Roadmap map = gridAt(950);
    Footprint footprint = new Footprint(map);
    assertThat(footprint.collideMoving(
        moving(id(map, 1, 1), id(map, 1, 2), Roadmap.DIR_POS_X, true),
        resting(id(map, 2, 1), Roadmap.DIR_NONE, false)))
        .describedAs("an idle vehicle with no known heading must be assumed broadside")
        .isTrue();
  }
}
