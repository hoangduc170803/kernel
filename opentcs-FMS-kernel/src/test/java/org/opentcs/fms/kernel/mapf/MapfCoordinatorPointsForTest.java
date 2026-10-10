package org.opentcs.fms.kernel.mapf;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.lang.reflect.Field;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.Test;
import org.opentcs.components.kernel.services.PlantModelService;
import org.opentcs.data.model.Path;
import org.opentcs.data.model.PlantModel;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Pose;
import org.opentcs.data.model.Triple;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.kernel.mapf.adg.AdgService;

class MapfCoordinatorPointsForTest {

  @Test
  void servesTheTailOfTheFleetRouteFromTheVehiclesCurrentPoint() {
    MapfCoordinator coordinator = coordinatorFor(line());
    coordinator.resolve(Map.of("V1", "A"), Map.of("V1", "D"));

    assertThat(coordinator.pointsFor("V1", "B", "D", ClaimPolicy.DEFAULT))
        .as("the vehicle already stands at B, so the leg still to drive is B..D and openTCS"
            + " must not be handed the A..D route the fleet was solved from")
        .contains(List.of("B", "C", "D"));
  }



  @Test
  void completesAFleetRouteThatStopsShortOfTheGoalItWasAimedAt() throws Exception {
    MapfCoordinator coordinator = coordinatorFor(line());
    installFleetPlanNoToySolveCanProduce(coordinator, "V1",
        new MapfCoordinator.PlannedRoute(List.of("A", "B"), "D", "D"));

    assertThat(coordinator.pointsFor("V1", "A", "D", ClaimPolicy.DEFAULT))
        .as("the fleet plan was aimed at D but only reaches B; the coordinated prefix is kept"
            + " and B..D is completed alone rather than the whole plan being thrown away")
        .contains(List.of("A", "B", "C", "D"));
  }

  @Test
  void refusesToServeAFleetRouteThatWasSolvedForADifferentGoal() {
    MapfCoordinator coordinator = coordinatorFor(line());
    coordinator.resolve(Map.of("V1", "B"), Map.of("V1", "D"));

    assertThat(coordinator.pointsFor("V1", "B", "A", ClaimPolicy.DEFAULT))
        .as("the fleet was solved for D; A is a new leg, so the stale route must not be"
            + " served just because the vehicle still stands on its first point - and there is"
            + " nothing else to serve until a solve aims at A")
        .isEmpty();
  }

  @Test
  void holdsBackWhenTheVehicleIsAbsentFromTheFleetPlan() {
    MapfCoordinator coordinator = coordinatorFor(line());

    assertThat(coordinator.pointsFor("V9", "A", "C", ClaimPolicy.DEFAULT))
        .as("no solve has covered V9, so there is no coordinated route to serve; a solo path"
            + " would ignore every other vehicle")
        .isEmpty();
  }

  @Test
  void holdsBackWhenTheVehicleIsNotOnItsFleetRoute() {
    MapfCoordinator coordinator = coordinatorFor(line());
    coordinator.resolve(Map.of("V1", "A"), Map.of("V1", "C"));

    assertThat(coordinator.pointsFor("V1", "D", "C", ClaimPolicy.DEFAULT))
        .as("the fleet routed V1 A..C, which never passes through D, so the plan says nothing"
            + " about where V1 stands now")
        .isEmpty();
  }

  @Test
  void holdsBackWhenOnlyTheLastPointOfTheFleetRouteIsLeft() {
    MapfCoordinator coordinator = coordinatorFor(line());
    coordinator.resolve(Map.of("V1", "A"), Map.of("V1", "C"));

    assertThat(coordinator.pointsFor("V1", "C", "D", ClaimPolicy.DEFAULT))
        .as("a one-point tail is not a drivable route, and driving around it alone would ignore"
            + " every other vehicle, so nothing is served until the next solve")
        .isEmpty();
  }

  @Test
  void servesNothingWhenEvenTheSingleVehiclePathIsShorterThanTwoPoints() {
    MapfCoordinator coordinator = coordinatorFor(line());
    coordinator.resolve(Map.of("V1", "A"), Map.of("V1", "C"));

    assertThat(coordinator.pointsFor("V1", "C", "C", ClaimPolicy.DEFAULT))
        .as("openTCS forbids a zero-step route, so a vehicle standing on its goal gets none")
        .isEmpty();
  }



  @Test
  void onlyLaterLegsMayStillBePlannedAlone() {
    MapfCoordinator coordinator = coordinatorFor(line());
    coordinator.resolve(Map.of("V1", "A"), Map.of("V1", "C"));

    assertThat(coordinator.pointsToDrive(
        "V1", "C", "D", new MapfCoordinator.DriveOrderLeg(1, 2), ClaimPolicy.DEFAULT))
        .as("the first leg is what puts the vehicle on the floor among the others, so without a"
            + " coordinated route it waits for the next solve")
        .isEmpty();
    assertThat(coordinator.pointsToDrive(
        "V1", "C", "D", new MapfCoordinator.DriveOrderLeg(2, 2), ClaimPolicy.DEFAULT))
        .as("a later leg sits past the planning window and is replaced before the vehicle gets"
            + " there, so it may still be planned alone rather than blocking the whole order")
        .contains(List.of("C", "D"));
  }

  private static void assertSameRouteWhicheverLegAsks(
      MapfCoordinator coordinator, String vehicle, String from, String goal) {
    Optional<List<String>> served = coordinator.pointsFor(vehicle, from, goal, ClaimPolicy.DEFAULT);
    for (int total = 1; total <= 3; total++) {
      for (int leg = 1; leg <= total; leg++) {
        assertThat(coordinator.pointsToDrive(
            vehicle, from, goal, new MapfCoordinator.DriveOrderLeg(leg, total),
            ClaimPolicy.DEFAULT))
            .as("%s %s->%s on leg %d of %d: the leg number decides how loudly the coordinator"
                + " speaks, never which points it hands openTCS", vehicle, from, goal, leg, total)
            .isEqualTo(served);
      }
    }
  }

  private static void installFleetPlanNoToySolveCanProduce(
      MapfCoordinator coordinator, String vehicle, MapfCoordinator.PlannedRoute planned)
      throws Exception {
    Field routeCache = MapfCoordinator.class.getDeclaredField("routeCache");
    routeCache.setAccessible(true);
    routeCache.set(coordinator, Map.of(vehicle, planned));
  }

  private static MapfCoordinator coordinatorFor(PlantModel model) {
    PlantModelService plantModelService = mock(PlantModelService.class);
    when(plantModelService.getPlantModel()).thenReturn(model);
    MapfCoordinator coordinator = new MapfCoordinator(plantModelService, new AdgService());
    AtomicLong clock = new AtomicLong();
    coordinator.setClock(() -> clock.addAndGet(2_000L));
    return coordinator;
  }

  private static PlantModel line() {
    Point a = point("A", 0, 0);
    Point b = point("B", 1, 0);
    Point c = point("C", 2, 0);
    Point d = point("D", 3, 0);
    return new PlantModel("line")
        .withPoints(Set.of(a, b, c, d))
        .withPaths(Set.of(path("A-B", a, b), path("B-C", b, c), path("C-D", c, d)));
  }

  private static PlantModel tJunction() {
    Point far = point("L2", 0, 0);
    Point near = point("L1", 1, 0);
    Point mid = point("MID", 2, 0);
    Point goal = point("GOAL", 3, 0);
    Point bay = point("BAY", 2, 1);
    return new PlantModel("tee")
        .withPoints(Set.of(far, near, mid, goal, bay))
        .withPaths(Set.of(
            path("L1-MID", near, mid),
            path("L2-L1", far, near),
            path("MID-BAY", mid, bay),
            path("MID-GOAL", mid, goal)));
  }

  private static Point point(String name, long x, long y) {
    return new Point(name).withPose(new Pose(new Triple(x, y, 0), 0.0));
  }

  private static Path path(String name, Point src, Point dst) {
    return new Path(name, src.getReference(), dst.getReference())
        .withLength(1000)
        .withMaxVelocity(1000)
        .withMaxReverseVelocity(1000);
  }
}
