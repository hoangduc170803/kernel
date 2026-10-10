package org.opentcs.fms.routing;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.ShuttleCycleDetector.State;

class ShuttleCycleDetectorTest {

  @Test
  void detectsOnlyMovedReturnsWithAnUnchangedNeighborhood() {
    Roadmap roadmap = new Roadmap(new Roadmap.Model(
        List.of(
            new Roadmap.Point("0134", 0, 0, 0),
            new Roadmap.Point("0144", 1000, 0, 0),
            new Roadmap.Point("0135", 0, 1000, 0),
            new Roadmap.Point("0136", 0, 2000, 0)),
        List.of(
            new Roadmap.Path("0134-0144", "0134", "0144", 1000, true),
            new Roadmap.Path("0134-0135", "0134", "0135", 1000, true))));
    int[] goals = { roadmap.idOf("0144"), roadmap.idOf("0136") };
    int[] axes = { Roadmap.AXIS_Y, Roadmap.AXIS_X };
    State root = new State(0,
        new int[] { roadmap.idOf("0134"), roadmap.idOf("0135") }, axes);
    State middle = new State(1,
        new int[] { roadmap.idOf("0144"), roadmap.idOf("0135") }, axes);
    State returned = new State(2,
        new int[] { roadmap.idOf("0134"), roadmap.idOf("0135") }, axes);

    assertNotNull(ShuttleCycleDetector.detect(
        roadmap, null, goals, 0, returned, middle, List.of(middle, root),
        StaticBodies.NONE, 3));

    State waiting = new State(2, middle.config(), middle.axis());
    assertNull(ShuttleCycleDetector.detect(
        roadmap, null, goals, 0, waiting, middle, List.of(root), StaticBodies.NONE, 3));

    State neighborMoved = new State(2,
        new int[] { roadmap.idOf("0134"), roadmap.idOf("0136") }, axes);
    assertNull(ShuttleCycleDetector.detect(
        roadmap, null, goals, 0, neighborMoved, middle, List.of(root),
        StaticBodies.NONE, 3));
  }
}
