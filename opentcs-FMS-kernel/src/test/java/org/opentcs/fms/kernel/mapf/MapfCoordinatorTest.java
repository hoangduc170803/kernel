package org.opentcs.fms.kernel.mapf;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.components.kernel.services.PlantModelService;
import org.opentcs.data.model.Path;
import org.opentcs.data.model.PlantModel;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Pose;
import org.opentcs.data.model.Triple;
import org.opentcs.fms.kernel.mapf.adg.AdgService;

class MapfCoordinatorTest {

  private final Point a = point("A", 0);
  private final Point b = point("B", 1);
  private final Point c = point("C", 2);
  private final Point d = point("D", 3);
  private final PlantModel model = new PlantModel("m")
      .withPoints(Set.of(a, b, c, d))
      .withPaths(Set.of(path("A-B", a, b), path("B-C", b, c), path("C-D", c, d)));

  private MapfCoordinator coordinator;

  @BeforeEach
  void setUp() {
    PlantModelService plantModelService = mock(PlantModelService.class);
    when(plantModelService.getPlantModel()).thenReturn(model);
    coordinator = new MapfCoordinator(plantModelService, new AdgService());
    AtomicLong vclock = new AtomicLong();
    coordinator.setClock(() -> vclock.addAndGet(2_000L));
  }

  @Test
  void aRerouteFromTheFrontierDoesNotReplayTheCommittedReversal() {
    MapfCoordinator.PlannedRoute planned = new MapfCoordinator.PlannedRoute(
        List.of("0510", "0515", "0510", "0531", "0001"), "0001", "0001",
        List.of("0510", "0515", "0510"));

    assertThat(planned.tailFrom("0510"))
        .as("openTCS asks for the route from the point after the sent steps, the second 0510;"
            + " serving the first one makes the merger append 0515 -> 0510 a second time")
        .contains(List.of("0510", "0531", "0001"));
  }

  @Test
  void aRouteWithoutCommitmentStillStartsAtTheFirstMatch() {
    MapfCoordinator.PlannedRoute planned = new MapfCoordinator.PlannedRoute(
        List.of("A", "B", "C", "D"), "D", "D");

    assertThat(planned.tailFrom("B")).contains(List.of("B", "C", "D"));
  }

  @Test
  void cacheIsEmptyBeforeAnySolve() {
    assertThat(coordinator.cachedRoute("V1", "C")).isEmpty();
  }

  @Test
  void planSingleReturnsShortestPath() {
    assertThat(coordinator.planSingle("V1", "A", "C", ClaimPolicy.DEFAULT))
        .containsExactly("A", "B", "C");
  }

  @Test
  void jointSolveCachesCoordinatedRoute() {
    coordinator.resolve(Map.of("V1", "A"), Map.of("V1", "C"));

    assertThat(coordinator.cachedRoute("V1", "C")).contains(java.util.List.of("A", "B", "C"));
  }

  @Test
  void cachedRouteMatchesOnGoalAndVehicle() {
    coordinator.resolve(Map.of("V1", "A"), Map.of("V1", "C"));

    assertThat(coordinator.cachedRoute("V1", "B")).isEmpty(); // wrong goal
    assertThat(coordinator.cachedRoute("V2", "C")).isEmpty(); // unknown vehicle
  }

  @Test
  void resolveWithEmptyFleetDoesNothing() {
    coordinator.resolve(Map.of(), Map.of());
    assertThat(coordinator.cachedRoute("V1", "C")).isEmpty();
  }

  @Test
  void reSolveRefreshesTheCache() {
    coordinator.resolve(Map.of("V1", "A"), Map.of("V1", "C"));
    assertThat(coordinator.cachedRoute("V1", "C")).isPresent();

    coordinator.resolve(Map.of("V1", "A"), Map.of("V1", "B"));
    assertThat(coordinator.cachedRoute("V1", "B")).contains(java.util.List.of("A", "B"));
    assertThat(coordinator.cachedRoute("V1", "C")).isEmpty(); // replaced by the new solve
  }

  @Test
  void aCommittedPlanIsLeftUntouchedWhenThePolicyDeclinesToReSolve() {
    assertThat(coordinator.resolve(Map.of("V1", "A"), Map.of("V1", "C"))).isTrue();
    long gen = coordinator.adg().generation();

    assertThat(coordinator.resolve(Map.of("V1", "A"), Map.of("V1", "C"))).isFalse();
    assertThat(coordinator.adg().generation())
        .as("a declined re-solve only touches the built-at stamp, so downstream sees the same"
            + " generation and nothing reroutes")
        .isEqualTo(gen);
    assertThat(coordinator.cachedRoute("V1", "C"))
        .as("and the route served to the fleet is the committed one, not a cleared cache")
        .contains(java.util.List.of("A", "B", "C"));
  }

  @Test
  void aNewPrecedenceGenerationIsInstalledWhenThePolicyCallsForAReSolve() {
    assertThat(coordinator.resolve(Map.of("V1", "A"), Map.of("V1", "C"))).isTrue();
    long gen = coordinator.adg().generation();

    assertThat(coordinator.resolve(Map.of("V1", "A"), Map.of("V1", "B"))).isTrue();
    assertThat(coordinator.adg().generation()).isGreaterThan(gen);
  }

  @Test
  void aPlanIsStillInstalledWhenAVehicleOnlyMovedFORWARDAlongTheRouteItWasGiven() {
    MapfCoordinator.SolveRequest request = coordinator.prepare(Map.of("V1", "A"), Map.of("V1", "C"), Set.of(),
        ClaimPolicy.DEFAULT, BodyState.UNKNOWN);
    MapfPlanner.PlanResult result = coordinator.compute(request);

    assertThat(coordinator.install(request, result))
        .as("a vehicle advancing while the solver ran is the NORMAL case; the plan was solved"
            + " from its frontier, so its progress is exactly what was already committed")
        .isTrue();
    assertThat(coordinator.cachedRoute("V1", "C")).contains(List.of("A", "B", "C"));
  }

  @Test
  void aPlanIsStillInstalledWhenAGoalChangedWhileItWasBeingSolved() {
    MapfCoordinator.SolveRequest request = coordinator.prepare(Map.of("V1", "A"), Map.of("V1", "C"), Set.of(),
        ClaimPolicy.DEFAULT, BodyState.UNKNOWN);
    MapfPlanner.PlanResult result = coordinator.compute(request);

    assertThat(coordinator.install(request, result))
        .as("goals change constantly in a warehouse — that is what lifelong MAPF is for. Throwing"
            + " the whole fleet's plan away because one vehicle was reassigned leaves the other"
            + " vehicles on an even older plan, and leaves precedence unrefreshed")
        .isTrue();
    assertThat(coordinator.cachedRoute("V1", "B"))
        .as("the reassigned vehicle is served by the requestedGoal mismatch in points(), which"
            + " falls back to a single-vehicle route until the next solve")
        .isEmpty();
  }

  @Test
  void aPlanIsDiscardedWhenThePlantModelWasReloadedWhileItWasBeingSolved() {
    MapfCoordinator.SolveRequest request = coordinator.prepare(Map.of("V1", "A"), Map.of("V1", "C"), Set.of(),
        ClaimPolicy.DEFAULT, BodyState.UNKNOWN);
    MapfPlanner.PlanResult result = coordinator.compute(request);
    coordinator.invalidateModel();

    assertThat(coordinator.install(request, result))
        .as("invalidateModel() WIPED routeCache and precedence on purpose; installing a plan"
            + " solved on the dead roadmap would undo that wipe. This is the one case where the"
            + " old state was not merely older but deliberately discarded")
        .isFalse();
  }

  @Test
  void theSolveSlotIsReleasedAgainAfterEveryOutcome() {
    MapfCoordinator.SolveRequest request = coordinator.prepare(Map.of("V1", "A"), Map.of("V1", "C"), Set.of(),
        ClaimPolicy.DEFAULT, BodyState.UNKNOWN);
    assertThat(coordinator.prepare(Map.of("V1", "A"), Map.of("V1", "C"), Set.of(),
        ClaimPolicy.DEFAULT, BodyState.UNKNOWN))
        .as("a second solve must not start while the first one holds the slot")
        .isNull();

    MapfPlanner.PlanResult result = coordinator.compute(request);
    coordinator.invalidateModel();
    assertThat(coordinator.install(request, result)).isFalse();

    assertThat(coordinator.prepare(Map.of("V1", "A"), Map.of("V1", "C"), Set.of(),
        ClaimPolicy.DEFAULT, BodyState.UNKNOWN))
        .as("a DISCARDED plan must still hand the slot back, or the fleet never replans again")
        .isNotNull();
  }

  @Test
  void abortingASolveHandsTheSlotBack() {
    coordinator.prepare(Map.of("V1", "A"), Map.of("V1", "C"), Set.of(),
        ClaimPolicy.DEFAULT, BodyState.UNKNOWN);

    coordinator.abortSolve("the solver threw");

    assertThat(coordinator.prepare(Map.of("V1", "A"), Map.of("V1", "C"), Set.of(),
        ClaimPolicy.DEFAULT, BodyState.UNKNOWN))
        .isNotNull();
  }

  private static Point point(String name, long x) {
    return new Point(name).withPose(new Pose(new Triple(x, 0, 0), 0.0));
  }

  private static Path path(String name, Point src, Point dst) {
    return new Path(name, src.getReference(), dst.getReference())
        .withLength(1000)
        .withMaxVelocity(1000)
        .withMaxReverseVelocity(1000);
  }

}
