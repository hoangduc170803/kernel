package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class LaneAxisTest {

  private static final double PITCH = 950;
  private static final int MAINLINE_LENGTH = 10;
  private static final int LANE_DEPTH = 4;

  private record Warehouse(List<Roadmap.Point> points, List<Roadmap.Path> paths) {
  }

  private static Warehouse warehouse() {
    List<Roadmap.Point> points = new ArrayList<>();
    List<Roadmap.Path> paths = new ArrayList<>();
    for (int row = 0; row < 2; row++) {
      for (int index = 0; index < MAINLINE_LENGTH; index++) {
        points.add(new Roadmap.Point(row + "m" + index, index * PITCH, row * PITCH, 0));
        if (index > 0) {
          paths.add(new Roadmap.Path(
              "e", row + "m" + (index - 1), row + "m" + index, PITCH, true));
        }
      }
    }
    for (int column : new int[] { 2, 5, 8 }) {
      String previous = "0m" + column;
      for (int depth = 1; depth <= LANE_DEPTH; depth++) {
        String name = "l" + column + "_" + depth;
        points.add(new Roadmap.Point(name, column * PITCH, -depth * PITCH, 0));
        paths.add(new Roadmap.Path("e", previous, name, PITCH, true));
        previous = name;
      }
    }
    return new Warehouse(points, paths);
  }

  private static Warehouse turnedAQuarter(Warehouse upright) {
    List<Roadmap.Point> turned = new ArrayList<>();
    for (Roadmap.Point point : upright.points()) {
      turned.add(new Roadmap.Point(point.name(), point.y(), point.x(), point.z()));
    }
    return new Warehouse(turned, upright.paths());
  }

  @Test
  void aLayoutThatDeclaresItsDirectionIsTakenAtItsWord() {
    Warehouse map = warehouse();

    assertThat(LaneAxis.of(map.points(), map.paths(), true))
        .as("v7-map-xuong carries property 'direction' on its visual layout, and that is the"
            + " one statement the geometry never has to be second-guessed against")
        .isEqualTo(Roadmap.AXIS_Y);
  }

  @Test
  void anUndeclaredLayoutIsReadOffItsLongestRunOfPoints() {
    Warehouse map = warehouse();

    assertThat(LaneAxis.of(map.points(), map.paths(), false))
        .as("the through-lines run along x and the drop-off lanes hang off them, so the lanes"
            + " are the perpendicular axis")
        .isEqualTo(Roadmap.AXIS_Y);
  }

  @Test
  void theSameWarehouseDrawnAQuarterTurnRoundReportsTheOtherAxis() {
    Warehouse turned = turnedAQuarter(warehouse());

    assertThat(LaneAxis.of(turned.points(), turned.paths(), false))
        .as("v7-aubot is v7-map-xuong rotated, so pinning the lane axis to y would have sent a"
            + " vehicle leaving a lane deeper into it instead of out across")
        .isEqualTo(Roadmap.AXIS_X);
  }

  @Test
  void crossingTheLaneIsTheMoveThatLeavesIt() {
    Warehouse map = warehouse();
    Roadmap upright = new Roadmap(new Roadmap.Model(map.points(), map.paths()));
    int insideALane = upright.idOf("l5_2");

    assertThat(upright.crossesLane(insideALane, upright.idOf("l5_3")))
        .as("deeper down the same lane is along it, never across")
        .isFalse();
    assertThat(upright.crossesLane(upright.idOf("0m5"), upright.idOf("0m6")))
        .as("a step along the through-line is the across-lane direction")
        .isTrue();
  }
}
