package org.opentcs.fms.routing.pibt;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.DistTable;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.StaticBodies;

class EnvironmentalObstacleGateTest {

  private static final int PITCH = 800;
  private static final int ONLY_AGENT = 0;
  private static final double CARGO_HALF_LENGTH = 400;
  private static final double CARGO_HALF_WIDTH = 370;

  @BeforeEach
  void enableFootprint() {
    System.setProperty("fms.pibt.footprint", "true");
  }

  @AfterEach
  void restoreDefaults() {
    System.clearProperty("fms.pibt.footprint");
  }

  private static Roadmap column() {
    List<Roadmap.Point> points = List.of(
        new Roadmap.Point("south", 0, 0, 0),
        new Roadmap.Point("middle", 0, PITCH, 0),
        new Roadmap.Point("north", 0, 2 * PITCH, 0));
    List<Roadmap.Path> paths = List.of(
        new Roadmap.Path("a", "south", "middle", PITCH, true),
        new Roadmap.Path("b", "middle", "north", PITCH, true));
    return new Roadmap(new Roadmap.Model(points, paths));
  }

  private static Footprint.Obstacle cargoOn(Roadmap map, String cell) {
    double x = map.xOf(map.idOf(cell));
    double y = map.yOf(map.idOf(cell));
    return new Footprint.Obstacle(
        "cargo@" + cell,
        x - CARGO_HALF_WIDTH, x + CARGO_HALF_WIDTH,
        y - CARGO_HALF_LENGTH, y + CARGO_HALF_LENGTH);
  }

  private static int[] step(Roadmap map, String from, String goal, StaticBodies statics) {
    int[] goals = {map.idOf(goal)};
    return new PibtEngine(map, 1, 0, DistTable.turnAware(map, goals)).step(
        new int[] {map.idOf(from)},
        new int[] {ONLY_AGENT},
        List.of(),
        new int[] {Roadmap.DIR_POS_Y},
        new boolean[] {true},
        statics,
        null);
  }

  @Test
  void withNoObstacleTheAgentWalksUpTheColumn() {
    Roadmap map = column();

    int[] next = step(map, "south", "north", StaticBodies.NONE);

    assertThat(map.nameOf(next[ONLY_AGENT])).isEqualTo("middle");
  }

  @Test
  void deliveredCargoBlocksTheCellBesideItNotJustTheOneItStandsOn() {
    Roadmap map = column();
    StaticBodies cargoAtNorth = StaticBodies.NONE.withObstacles(List.of(cargoOn(map, "north")));

    int[] next = step(map, "south", "north", cargoAtNorth);

    assertThat(map.nameOf(next[ONLY_AGENT]))
        .as("a loaded body at 'middle' reaches 420 + 50mm towards north, the cargo reaches 400mm"
            + " back, and 870mm exceeds the 800mm pitch")
        .isEqualTo("south");
  }

  @Test
  void theVehicleThatJustDroppedTheCargoMayBackAwayFromIt() {
    Roadmap map = column();
    StaticBodies cargoAtNorth = StaticBodies.NONE.withObstacles(List.of(cargoOn(map, "north")));

    int[] next = step(map, "north", "south", cargoAtNorth);

    assertThat(map.nameOf(next[ONLY_AGENT])).isEqualTo("middle");
  }

  @Test
  void aVehicleAlreadyTouchingTheCargoMayNotMoveCloserToIt() {
    Roadmap map = column();
    Footprint footprint = new Footprint(map);
    Footprint.Obstacle cargo = cargoOn(map, "north");

    Footprint.Motion closer = new Footprint.Motion(
        map.idOf("middle"), map.idOf("north"), Roadmap.DIR_POS_Y, true);
    Footprint.Motion waiting = Footprint.Motion.resting(
        map.idOf("middle"), Roadmap.DIR_POS_Y, true);

    assertThat(footprint.collideMoving(closer, cargo)).isTrue();
    assertThat(footprint.collideMoving(waiting, cargo)).isFalse();
  }
}
