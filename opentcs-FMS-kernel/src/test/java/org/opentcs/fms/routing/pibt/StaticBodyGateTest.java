package org.opentcs.fms.routing.pibt;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.DistTable;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.StaticBodies;

class StaticBodyGateTest {

  private static final int PITCH = 750;
  private static final int ONLY_AGENT = 0;

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

  private static int[] stepTowardsNorth(Roadmap map, StaticBodies statics) {
    int[] goals = {map.idOf("north")};
    return new PibtEngine(map, 1, 0, DistTable.turnAware(map, goals)).step(
        new int[] {map.idOf("south")},
        new int[] {ONLY_AGENT},
        List.of(),
        new int[] {Roadmap.DIR_POS_Y},
        new boolean[] {true},
        statics,
        null);
  }

  @Test
  void withNoStaticBodyTheAgentWalksStraightUpTheColumn() {
    Roadmap map = column();

    int[] next = stepTowardsNorth(map, StaticBodies.NONE);

    assertThat(map.nameOf(next[ONLY_AGENT])).isEqualTo("middle");
  }

  @Test
  void aParkedVehicleBlocksTheCellBesideItNotJustTheOneItStandsOn() {
    Roadmap map = column();
    StaticBodies parkedAtNorth = new StaticBodies(
        new int[] {map.idOf("north")},
        new int[] {Roadmap.DIR_POS_Y},
        new boolean[] {true});

    int[] next = stepTowardsNorth(map, parkedAtNorth);

    assertThat(map.nameOf(next[ONLY_AGENT]))
        .as("the parked body is 840mm along the column and so is the mover's, so the two need"
            + " 890mm with clearance while the cells are 750mm apart; masking only the cell the"
            + " parked vehicle stands on leaves 'middle' looking free and drives a body into it")
        .isEqualTo("south");
  }

  @Test
  void anEmptyParkedVehicleLyingAcrossTheColumnLeavesTheCellUsable() {
    Roadmap map = column();
    StaticBodies parkedBroadside = new StaticBodies(
        new int[] {map.idOf("north")},
        new int[] {Roadmap.DIR_POS_X},
        new boolean[] {false});

    int[] next = stepTowardsNorth(map, parkedBroadside);

    assertThat(map.nameOf(next[ONLY_AGENT]))
        .as("an empty body turned across the column reaches only 212.5mm towards it, so 420 +"
            + " 212.5 + 50 stays under the 750mm pitch and the cell is genuinely free")
        .isEqualTo("middle");
  }
}
