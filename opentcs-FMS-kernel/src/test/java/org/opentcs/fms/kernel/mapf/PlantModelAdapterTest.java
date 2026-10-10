package org.opentcs.fms.kernel.mapf;

import static org.assertj.core.api.Assertions.assertThat;

import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.util.Set;
import org.opentcs.fms.kernel.integration.PlantModelXmlLoader;
import org.junit.jupiter.api.Test;
import org.opentcs.data.model.Path;
import org.opentcs.data.model.PlantModel;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Pose;
import org.opentcs.data.model.Triple;
import org.opentcs.fms.routing.Roadmap;

class PlantModelAdapterTest {

  private static Point point(String name, long x, long y) {
    return new Point(name).withPose(new Pose(new Triple(x, y, 0), 0.0));
  }

  private static Path path(String name, Point src, Point dst, int fwd, int rev, boolean locked) {
    return new Path(name, src.getReference(), dst.getReference())
        .withLength(1000)
        .withMaxVelocity(fwd)
        .withMaxReverseVelocity(rev)
        .withLocked(locked);
  }

  @Test
  void theRealMapSplitsDropOffSlotsFromPickupSlots() throws Exception {
    File map = PlantModelXmlLoader.findMap("v7-vda5050-test.xml");
    assumeTrue(map != null, "v7-vda5050-test.xml not found; skipping");
    Roadmap.Model model = PlantModelAdapter.toModel(PlantModelXmlLoader.load(map));

    assertThat(model.dropOffPoints())
        .as("3098 and 3118 flank the shared aisle cell 3108 and are both Drop off slots")
        .contains("3098", "3118");
    assertThat(model.dropOffPoints())
        .as("0104 and 0164 are Pick up slots, so a vehicle collecting there must not be"
            + " mistaken for one loitering in a drop-off block")
        .doesNotContain("0104", "0164");
    assertThat(model.cargoSlotPoints())
        .as("a pick-up cell holds waiting cargo, so a vehicle that has no business there must be"
            + " priced off it")
        .contains("0104", "0164");
    assertThat(model.cargoSlotPoints())
        .as("a drop-off cell is a lane every vehicle has to drive through to reach the back of"
            + " the column and to back out again; charging the big-M there made a 10-cell lap"
            + " round the block cheaper than the two-cell retreat straight up the column")
        .doesNotContain("3098", "3118");
    assertThat(model.dropOffPoints()).hasSize(144);
    assertThat(model.cargoSlotPoints()).hasSize(80);
  }

  @Test
  void honoursDirectionalityAndSkipsLocked() {
    Point a = point("A", 0, 0);
    Point b = point("B", 1, 0);
    Point c = point("C", 2, 0);

    PlantModel model = new PlantModel("test")
        .withPoints(Set.of(a, b, c))
        .withPaths(Set.of(
            path("A-B", a, b, 1000, 1000, false), // two-way
            path("B-C", b, c, 1000, 0, false),    // one-way forward only
            path("A-C", a, c, 1000, 1000, true)   // locked -> skipped
        ));

    Roadmap r = PlantModelAdapter.toRoadmap(model);
    int ia = r.idOf("A");
    int ib = r.idOf("B");
    int ic = r.idOf("C");

    // A<->B is bidirectional.
    assertThat(r.distance(ia, ib)).isEqualTo(1);
    assertThat(r.distance(ib, ia)).isEqualTo(1);

    // B->C one-way: forward reachable, reverse not.
    assertThat(r.distance(ib, ic)).isEqualTo(1);
    assertThat(r.distance(ic, ib)).isGreaterThanOrEqualTo(Roadmap.INF);

    // A->C direct path is locked, but A->B->C still connects (cost 2);
    // C->A has no legal direction at all.
    assertThat(r.distance(ia, ic)).isEqualTo(2);
    assertThat(r.distance(ic, ia)).isGreaterThanOrEqualTo(Roadmap.INF);
  }

  @Test
  void mapsEveryPointToAVertex() {
    Point a = point("A", 0, 0);
    Point b = point("B", 1, 0);
    PlantModel model = new PlantModel("m")
        .withPoints(Set.of(a, b))
        .withPaths(Set.of(path("A-B", a, b, 1000, 0, false)));

    Roadmap r = PlantModelAdapter.toRoadmap(model);
    assertThat(r.size()).isEqualTo(2);
    assertThat(r.has("A")).isTrue();
    assertThat(r.has("B")).isTrue();
  }
}
