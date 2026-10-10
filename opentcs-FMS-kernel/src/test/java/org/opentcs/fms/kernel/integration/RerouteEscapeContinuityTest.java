package org.opentcs.fms.kernel.integration;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.opentcs.data.model.Path;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.DriveOrder;
import org.opentcs.data.order.Route;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.drivers.vehicle.MovementCommand;
import org.opentcs.drivers.vehicle.VehicleController;
import org.opentcs.drivers.vehicle.VehicleControllerPool;
import org.opentcs.fms.kernel.rerouting.FmsRegularDriveOrderMerger;

/**
 * Offline (no kernel) proof that the frontier-anchored {@link FmsRegularDriveOrderMerger}
 * lets a deadlock-escape route actually reach a committed vehicle -- the case the
 * hand-rolled {@link RerouteContinuityService} mirror marks as unsolvable.
 *
 * <p>Scenario: a vehicle sitting on B has already SENT the command B-&gt;C (its
 * commit frontier is C) on the old route B-&gt;C-&gt;D-&gt;E-&gt;G, but D is blocked
 * head-on. The coordinator's escape reverses it back out. Two models disagree:
 * <ul>
 *   <li>The harness mirror reroutes from the CURRENT point B with a coarse one-step
 *       rule and REJECTS the escape (its next step A != the committed next C) -- the
 *       vehicle keeps its deadlocked route.</li>
 *   <li>The real kernel reroutes from the FRONTIER C; the merger splices the escape
 *       AFTER the committed B-&gt;C, yielding B-&gt;C-&gt;B-&gt;A-&gt;X-&gt;G. Every
 *       already-sent step is preserved, so {@code driveOrdersContinual} passes and the
 *       escape IS applied -- the reversal is merely deferred past the frontier.</li>
 * </ul>
 */
class RerouteEscapeContinuityTest {

  @Test
  void frontierMergerDefersAReversalEscapePastTheCommittedStep() {
    DriveOrder committed = driveOrder("B", "C", "D", "E", "G");
    DriveOrder escape = driveOrder("C", "B", "A", "X", "G");
    int frontierIndex = 0;

    Vehicle vehicle = new Vehicle("V");
    FmsRegularDriveOrderMerger merger
        = new FmsRegularDriveOrderMerger(poolWithFrontier(vehicle, committed, frontierIndex));
    Route merged = merger.mergeDriveOrders(
        committed,
        escape,
        new TransportOrder("t", List.of()),
        TransportOrder.ROUTE_STEP_INDEX_DEFAULT,
        vehicle).getRoute();

    assertThat(points(merged))
        .as("escape spliced AFTER the committed frontier C")
        .containsExactly("B", "C", "B", "A", "X", "G");
    assertThat(committedPrefixPreserved(merged, committed, frontierIndex))
        .as("every already-sent step is preserved -> route is continual -> applicable")
        .isTrue();
  }

  private VehicleControllerPool poolWithFrontier(
      Vehicle vehicle, DriveOrder committed, int frontierIndex) {
    MovementCommand lastSent = mock(MovementCommand.class);
    when(lastSent.getStep()).thenReturn(committed.getRoute().getSteps().get(frontierIndex));
    VehicleController controller = mock(VehicleController.class);
    when(controller.getInteractionsPendingCommand()).thenReturn(Optional.empty());
    when(controller.getCommandsSent()).thenReturn(new LinkedList<>(List.of(lastSent)));
    VehicleControllerPool pool = mock(VehicleControllerPool.class);
    when(pool.getVehicleController(vehicle.getName())).thenReturn(controller);
    return pool;
  }

  private boolean committedPrefixPreserved(Route merged, DriveOrder committed, int frontierIndex) {
    List<Route.Step> committedSteps = committed.getRoute().getSteps();
    for (int i = 0; i <= frontierIndex; i++) {
      Route.Step a = committedSteps.get(i);
      Route.Step b = merged.getSteps().get(i);
      if (!Objects.equals(a.getSourcePoint(), b.getSourcePoint())
          || !Objects.equals(a.getDestinationPoint(), b.getDestinationPoint())
          || !Objects.equals(a.getPath(), b.getPath())) {
        return false;
      }
    }
    return true;
  }

  private static List<String> points(Route route) {
    List<String> pts = new ArrayList<>();
    for (Route.Step step : route.getSteps()) {
      if (pts.isEmpty() && step.getSourcePoint() != null) {
        pts.add(step.getSourcePoint().getName());
      }
      pts.add(step.getDestinationPoint().getName());
    }
    return pts;
  }

  private DriveOrder driveOrder(String startPoint, String... pointNames) {
    List<Point> points = new ArrayList<>();
    for (String pointName : pointNames) {
      points.add(new Point(pointName));
    }
    DriveOrder.Destination dest
        = new DriveOrder.Destination(points.get(points.size() - 1).getReference());
    return new DriveOrder("order", dest).withRoute(route(new Point(startPoint), points));
  }

  private Route route(Point startPoint, List<Point> points) {
    List<Route.Step> steps = new ArrayList<>();
    Point src = startPoint;
    for (int i = 0; i < points.size(); i++) {
      Point dst = points.get(i);
      Path path = new Path(src.getName() + " --- " + dst.getName(),
          src.getReference(), dst.getReference());
      steps.add(new Route.Step(path, src, dst, Vehicle.Orientation.FORWARD, i, 1));
      src = dst;
    }
    return new Route(steps);
  }
}
