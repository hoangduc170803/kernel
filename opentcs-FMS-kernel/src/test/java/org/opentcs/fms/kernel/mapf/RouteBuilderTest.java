package org.opentcs.fms.kernel.mapf;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.opentcs.data.model.Path;
import org.opentcs.data.model.PlantModel;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Pose;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.Route;

class RouteBuilderTest {

  private static Point point(String name, long x) {
    return new Point(name).withPose(new Pose(new Triple(x, 0, 0), 0.0));
  }

  private static Path path(String name, Point src, Point dst, int fwd, int rev) {
    return new Path(name, src.getReference(), dst.getReference())
        .withLength(1500)
        .withMaxVelocity(fwd)
        .withMaxReverseVelocity(rev);
  }

  private final Point a = point("A", 0);
  private final Point b = point("B", 1);
  private final Point c = point("C", 2);
  private final PlantModel model = new PlantModel("m")
      .withPoints(Set.of(a, b, c))
      .withPaths(Set.of(
          path("A-B", a, b, 1000, 1000), // two-way
          path("B-C", b, c, 1000, 0)));  // forward only

  @Test
  void buildsForwardSteps() {
    Route route = new RouteBuilder(model).buildRoute(List.of("A", "B", "C")).orElseThrow();

    assertThat(route.getSteps()).hasSize(2);
    Route.Step s0 = route.getSteps().get(0);
    assertThat(s0.getSourcePoint().getName()).isEqualTo("A");
    assertThat(s0.getDestinationPoint().getName()).isEqualTo("B");
    assertThat(s0.getVehicleOrientation()).isEqualTo(Vehicle.Orientation.FORWARD);
    assertThat(s0.getPath().getName()).isEqualTo("A-B");

    Route.Step s1 = route.getSteps().get(1);
    assertThat(s1.getSourcePoint().getName()).isEqualTo("B");
    assertThat(s1.getDestinationPoint().getName()).isEqualTo("C");
  }

  @Test
  void usesBackwardOrientationOnReverseTraversal() {
    // B -> A is the reverse of path "A-B" (legal because it is two-way).
    Route route = new RouteBuilder(model).buildRoute(List.of("B", "A")).orElseThrow();

    assertThat(route.getSteps()).hasSize(1);
    Route.Step step = route.getSteps().get(0);
    assertThat(step.getSourcePoint().getName()).isEqualTo("B");
    assertThat(step.getDestinationPoint().getName()).isEqualTo("A");
    assertThat(step.getVehicleOrientation()).isEqualTo(Vehicle.Orientation.BACKWARD);
    assertThat(step.getPath().getName()).isEqualTo("A-B");
  }

  @Test
  void stayPutHasNoRoute() {
    assertThat(new RouteBuilder(model).buildRoute(List.of("A"))).isEmpty();
  }

  @Test
  void rejectsIllegalReverseTraversal() {
    // C -> B is illegal: path "B-C" is forward only.
    assertThatThrownBy(() -> new RouteBuilder(model).buildRoute(List.of("C", "B")))
        .isInstanceOf(IllegalStateException.class);
  }
}
