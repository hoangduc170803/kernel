package org.opentcs.fms.kernel.mapf;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.StaticBodies;

class StaticBodiesTest {

  private static final int PITCH = 750;
  private static final String PARKED = "V-parked";
  private static final String TURNING = "V-turning";

  private static Roadmap lane() {
    List<Roadmap.Point> points = List.of(
        new Roadmap.Point("mouth", 0, 0, 0),
        new Roadmap.Point("middle", 0, PITCH, 0),
        new Roadmap.Point("tip", 0, 2 * PITCH, 0));
    List<Roadmap.Path> paths = List.of(
        new Roadmap.Path("a", "mouth", "middle", PITCH, true),
        new Roadmap.Path("b", "middle", "tip", PITCH, true));
    return new Roadmap(new Roadmap.Model(points, paths));
  }

  private static Map<String, String> heldAt(String... vehicleThenPoint) {
    Map<String, String> held = new LinkedHashMap<>();
    for (int i = 0; i < vehicleThenPoint.length; i += 2) {
      held.put(vehicleThenPoint[i], vehicleThenPoint[i + 1]);
    }
    return held;
  }

  @Test
  void aFleetWithNothingHeldProducesNoBodiesAtAll() {
    assertThat(MapfPlanner.staticBodiesOf(lane(), Map.of(), BodyState.UNKNOWN))
        .isSameAs(StaticBodies.NONE);
  }

  @Test
  void everyHeldVehicleBecomesABodyCarryingItsOwnHeadingAndLoad() {
    Roadmap map = lane();
    BodyState bodies = new BodyState(
        Map.of(PARKED, Roadmap.DIR_POS_Y, TURNING, Roadmap.DIR_NEG_X),
        Set.of(TURNING));

    StaticBodies statics = MapfPlanner.staticBodiesOf(
        map, heldAt(PARKED, "middle", TURNING, "tip"), bodies);

    assertThat(statics.size()).isEqualTo(2);
    assertThat(statics.cells()).containsExactly(map.idOf("middle"), map.idOf("tip"));
    assertThat(statics.noses()).containsExactly(Roadmap.DIR_POS_Y, Roadmap.DIR_NEG_X);
    assertThat(statics.loaded()).containsExactly(false, true);
  }

  @Test
  void aHeldVehicleWithNoKnownHeadingIsCarriedAsUnknownRatherThanGuessed() {
    Roadmap map = lane();

    StaticBodies statics = MapfPlanner.staticBodiesOf(
        map, heldAt(PARKED, "mouth"), BodyState.UNKNOWN);

    assertThat(statics.noses()).containsExactly(Roadmap.DIR_NONE);
    assertThat(statics.loaded()).containsExactly(false);
  }
}
