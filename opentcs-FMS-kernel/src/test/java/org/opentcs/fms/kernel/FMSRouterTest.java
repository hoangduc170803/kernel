package org.opentcs.fms.kernel;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.components.kernel.services.InternalTCSObjectService;
import org.opentcs.components.kernel.services.PlantModelService;
import org.opentcs.data.model.Location;
import org.opentcs.data.model.LocationType;
import org.opentcs.data.model.Path;
import org.opentcs.data.model.PlantModel;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Pose;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.DriveOrder;
import org.opentcs.data.order.Route;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.fms.kernel.mapf.MapfCoordinator;
import org.opentcs.fms.kernel.scheduling.RetreatHolds;
import org.opentcs.strategies.basic.routing.DefaultRouter;
import org.opentcs.fms.kernel.mapf.adg.AdgService;

class FMSRouterTest {

  private final Point a = point("A", 0);
  private final Point b = point("B", 1);
  private final Point c = point("C", 2);
  private final PlantModel model = new PlantModel("m")
      .withPoints(Set.of(a, b, c))
      .withPaths(Set.of(
          path("A-B", a, b),
          path("B-C", b, c)));

  private static final Point LANE_TOP = point("0511", 0, 1500);
  private static final Point LANE_MID = point("0516", 0, 750);
  private static final Point LANE_BOTTOM = point("0521", 0, 0);
  private static final Point SIDE_EXIT = point("0515", 950, 750);
  private static final Point CROSSROAD = point("0002", 3000, 750);
  private static final Point CROSSROAD_BELOW = point("0007", 3000, 0);
  private static final Point CROSSROAD_WEST = point("0001", 2050, 0);

  private static final Instant EARLIER = Instant.parse("2026-08-21T01:00:00Z");
  private static final Instant LATER = Instant.parse("2026-08-21T01:00:01Z");

  private DefaultRouter delegate;
  private InternalTCSObjectService objectService;
  private MapfCoordinator coordinator;
  private FMSRouter router;

  @BeforeEach
  void setUp() {
    delegate = mock(DefaultRouter.class);
    objectService = mock(InternalTCSObjectService.class);
    PlantModelService plantModelService = mock(PlantModelService.class);
    when(plantModelService.getPlantModel()).thenReturn(model);

    coordinator = new MapfCoordinator(plantModelService, new AdgService());
    router = new FMSRouter(delegate, objectService, coordinator);
    router.initialize();
  }

  @Test
  void holdsTheOrderBackUntilAFleetSolveCoversTheVehicle() {
    TransportOrder order = new TransportOrder(
        "TO-0", List.of(new DriveOrder("d0", new DriveOrder.Destination(c.getReference()))));

    Set<List<Route>> result = router.getRoutes(new Vehicle("V1"), a, order, 1);

    assertThat(result)
        .as("no solve has covered V1 yet, so there is no coordinated route to hand over; openTCS"
            + " is told nothing and retries next cycle rather than driving a solo path")
        .isEmpty();
    verify(delegate, never()).getRoutes(any(), any(), any(TransportOrder.class), anyInt());
  }

  @Test
  void servesAStayPutRouteWhenTheVehicleAlreadyStandsOnTheDestination() {
    TransportOrder order = new TransportOrder(
        "TO-5", List.of(new DriveOrder("d0", new DriveOrder.Destination(a.getReference()))));

    Set<List<Route>> result = router.getRoutes(new Vehicle("V1"), a, order, 1);

    List<Route.Step> steps = result.iterator().next().get(0).getSteps();
    assertThat(steps)
        .as("openTCS forbids a zero-step route, so standing still is expressed as one step"
            + " carrying no path - built here rather than delegated to the default router")
        .hasSize(1);
    assertThat(steps.get(0).getPath()).isNull();
    assertThat(steps.get(0).getDestinationPoint().getName()).isEqualTo("A");
    verify(delegate, never()).getRoutes(any(), any(), any(TransportOrder.class), anyInt());
  }

  @Test
  void injectsMapfRouteForTransportOrder() {
    coordinator.resolve(Map.of("V1", "A"), Map.of("V1", "C"));
    TransportOrder order = new TransportOrder(
        "TO-1", List.of(new DriveOrder("d0", new DriveOrder.Destination(c.getReference()))));

    Set<List<Route>> result = router.getRoutes(new Vehicle("V1"), a, order, 1);

    assertThat(result).hasSize(1);
    List<Route> chain = result.iterator().next();
    assertThat(chain).hasSize(1); // one Route for the single drive order
    List<Route.Step> steps = chain.get(0).getSteps();
    assertThat(steps).hasSize(2);
    assertThat(steps.get(0).getSourcePoint().getName()).isEqualTo("A");
    assertThat(steps.get(0).getDestinationPoint().getName()).isEqualTo("B");
    assertThat(steps.get(1).getDestinationPoint().getName()).isEqualTo("C");

    // The default router was bypassed.
    verify(delegate, never()).getRoutes(any(), any(), any(TransportOrder.class), anyInt());
  }

  @Test
  void servesNothingWhenTheGoalIsNotInTheRoadmap() {
    Point z = point("Z", 9); // not part of the plant model
    TransportOrder order = new TransportOrder(
        "TO-2", List.of(new DriveOrder("d0", new DriveOrder.Destination(z.getReference()))));

    Set<List<Route>> result = router.getRoutes(new Vehicle("V1"), a, order, 1);

    assertThat(result)
        .as("a destination the roadmap does not know cannot be reached under coordination, and"
            + " routing it around the fleet plan is exactly what we refuse to do")
        .isEmpty();
    verify(delegate, never()).getRoutes(any(), any(), any(TransportOrder.class), anyInt());
  }

  @Test
  void routesLocationDestinationToItsLinkedBay() {
    Point pA = point("A", 0);
    Point pB = point("B", 1);
    Point pC = point("C", 2);
    PlantModel locModel = new PlantModel("m2")
        .withPoints(Set.of(pA, pB, pC))
        .withPaths(Set.of(path("A-B", pA, pB), path("B-C", pB, pC)));

    DefaultRouter delegate2 = mock(DefaultRouter.class);
    InternalTCSObjectService obj = mock(InternalTCSObjectService.class);
    PlantModelService pms = mock(PlantModelService.class);
    when(pms.getPlantModel()).thenReturn(locModel);

    LocationType lt = new LocationType("lt");
    Location bare = new Location("L", lt.getReference());
    Location loc = bare.withAttachedLinks(Set.of(
        new Location.Link(bare.getReference(), pC.getReference())));
    when(obj.fetch(Location.class, "L")).thenReturn(Optional.of(loc));

    MapfCoordinator locCoordinator = new MapfCoordinator(pms, new AdgService());
    FMSRouter locRouter = new FMSRouter(delegate2, obj, locCoordinator);
    locRouter.initialize();
    locCoordinator.resolve(Map.of("V1", "A"), Map.of("V1", "C"));

    TransportOrder order = new TransportOrder(
        "TO-3", List.of(new DriveOrder("d0", new DriveOrder.Destination(loc.getReference()))));

    Set<List<Route>> result = locRouter.getRoutes(new Vehicle("V1"), pA, order, 1);

    assertThat(result).hasSize(1);
    List<Route.Step> steps = result.iterator().next().get(0).getSteps();
    assertThat(steps.get(steps.size() - 1).getDestinationPoint().getName()).isEqualTo("C");
    verify(delegate2, never()).getRoutes(any(), any(), any(TransportOrder.class), anyInt());
  }

  @Test
  void twoGoalsOneCellApartOnTheReversedPickupLaneCannotBothBeServed() {
    FMSRouter laneRouter = routerOverThePickupLane();
    TransportOrder driving = orderAimedAt("TO-early", LANE_MID, EARLIER)
        .withState(TransportOrder.State.BEING_PROCESSED);
    TransportOrder arriving = dropOffOrderAimedAt("TO-late", LANE_BOTTOM, LATER);
    liveOrders(driving, arriving);

    assertThat(laneRouter.checkGeneralRoutability(arriving))
        .as("0516 and 0521 sit 750mm apart and every path into either one runs along Y, so the"
            + " two bodies stand nose to nose needing 840mm of lane; no heading the map can"
            + " deliver lets them coexist")
        .isFalse();
    verify(objectService).updateObjectProperty(
        eq(arriving.getReference()), eq(FMSRouter.PROPKEY_UNROUTABLE_REASON),
        eq("goalFootprint:0521:0516:TO-early"));
    verify(delegate, never()).checkGeneralRoutability(arriving);
  }

  @Test
  void aPathThatOnlyLeadsOutOfTheCellNeverGivesTheVehicleASidewaysStance() {
    FMSRouter laneRouter = routerOverThePickupLane();
    TransportOrder driving = orderAimedAt("TO-early", LANE_MID, EARLIER)
        .withState(TransportOrder.State.BEING_PROCESSED);
    TransportOrder arriving = dropOffOrderAimedAt("TO-late", LANE_BOTTOM, LATER);
    liveOrders(driving, arriving);

    assertThat(laneRouter.checkGeneralRoutability(arriving))
        .as("0516 has a horizontal path to 0515, but it is outbound only, so it can never deliver"
            + " a vehicle into 0516 broadside; reading successors instead of predecessors would"
            + " invent that stance and wave this pair through")
        .isFalse();
  }

  @Test
  void aGoalTheMapCanEnterFromTwoAxesIsLeftForTheSolverToArrange() {
    FMSRouter laneRouter = routerOverThePickupLane();
    TransportOrder driving = orderAimedAt("TO-early", CROSSROAD_BELOW, EARLIER)
        .withState(TransportOrder.State.BEING_PROCESSED);
    TransportOrder arriving = dropOffOrderAimedAt("TO-late", CROSSROAD, LATER);
    liveOrders(driving, arriving);

    assertThat(laneRouter.checkGeneralRoutability(arriving))
        .as("0007 can be entered heading along X or along Y, so one vehicle can stand broadside"
            + " and the pair fits inside 750mm; refusing here would cancel work the solver can"
            + " actually serve")
        .isTrue();
    verify(delegate).checkGeneralRoutability(arriving);
  }

  @Test
  void theSameGoalTwiceIsRefusedAsADuplicateRatherThanAsGeometry() {
    FMSRouter laneRouter = routerOverThePickupLane();
    TransportOrder driving = orderAimedAt("TO-early", LANE_MID, EARLIER)
        .withState(TransportOrder.State.BEING_PROCESSED);
    TransportOrder arriving = dropOffOrderAimedAt("TO-late", LANE_MID, LATER);
    liveOrders(driving, arriving);

    assertThat(laneRouter.checkGeneralRoutability(arriving)).isFalse();
    verify(objectService).updateObjectProperty(
        eq(arriving.getReference()), eq(FMSRouter.PROPKEY_UNROUTABLE_REASON),
        eq("duplicateGoal:0516:TO-early"));
  }

  @Test
  void theOlderOrderSurvivesWhicheverWayTheRepositoryHappensToStreamThem() {
    FMSRouter laneRouter = routerOverThePickupLane();
    TransportOrder older = dropOffOrderAimedAt("TO-zulu", LANE_MID, EARLIER);
    TransportOrder newer = dropOffOrderAimedAt("TO-alpha", LANE_BOTTOM, LATER);

    for (List<TransportOrder> ordering : List.of(List.of(older, newer), List.of(newer, older))) {
      liveOrders(ordering.toArray(new TransportOrder[0]));

      assertThat(laneRouter.checkGeneralRoutability(older))
          .as("the repository hands out an unmodifiable Set whose iteration order is salted per"
              + " JVM start, so a rule that kept whoever happened to be visited first would"
              + " cancel a different order after every kernel restart; streamed as %s", ordering)
          .isTrue();
      assertThat(laneRouter.checkGeneralRoutability(newer)).isFalse();
    }
  }

  @Test
  void anOrderOnItsWayOutNoLongerHoldsTheCellItWasAimedAt() {
    FMSRouter laneRouter = routerOverThePickupLane();
    TransportOrder abandoned = orderAimedAt("TO-early", LANE_MID, EARLIER)
        .withState(TransportOrder.State.WITHDRAWN);
    TransportOrder arriving = dropOffOrderAimedAt("TO-late", LANE_BOTTOM, LATER);
    liveOrders(abandoned, arriving);

    assertThat(laneRouter.checkGeneralRoutability(arriving))
        .as("a withdrawn order is not driving anywhere, so holding its goal against new work"
            + " would strand the busiest cells")
        .isTrue();
  }

  @Test
  void onlyTheLegBeingDrivenCountsNotTheOnesAlreadyBehindTheVehicle() {
    FMSRouter laneRouter = routerOverThePickupLane();
    TransportOrder driving = new TransportOrder("TO-early", List.of(
        new DriveOrder("leg0", new DriveOrder.Destination(LANE_MID.getReference())),
        new DriveOrder("leg1", new DriveOrder.Destination(CROSSROAD.getReference()))))
        .withCreationTime(EARLIER)
        .withState(TransportOrder.State.BEING_PROCESSED)
        .withCurrentDriveOrderIndex(1);
    TransportOrder arriving = dropOffOrderAimedAt("TO-late", LANE_BOTTOM, LATER);
    liveOrders(driving, arriving);

    assertThat(laneRouter.checkGeneralRoutability(arriving))
        .as("the vehicle has left 0516 behind and is driving leg 1, so blocking 0521 against a"
            + " cell nobody is headed for would refuse work for no reason")
        .isTrue();
  }

  @Test
  void aDropOffOrderInheritsTheGoalItsOwnApproachOrderIsStillHolding() {
    FMSRouter laneRouter = routerOverThePickupLane();
    TransportOrder approach = legOrder("TO-early", LANE_MID, EARLIER, RetreatHolds.APPROACH_LEG)
        .withState(TransportOrder.State.BEING_PROCESSED)
        .withProcessingVehicle(new Vehicle("V1").getReference());
    TransportOrder dropoff = legOrder("TO-late", LANE_MID, LATER, RetreatHolds.DROPOFF_LEG)
        .withIntendedVehicle(new Vehicle("V1").getReference());
    liveOrders(approach, dropoff);

    assertThat(laneRouter.checkGeneralRoutability(dropoff))
        .as("WES issues the drop-off order before it withdraws the approach order, and keeps the"
            + " approach order alive when it already points at the committed slot, so the pair"
            + " shares a goal on purpose; one vehicle cannot clash with itself")
        .isTrue();
    verify(delegate).checkGeneralRoutability(dropoff);
  }

  @Test
  void theApproachOrderOfAnotherVehicleStillHoldsTheCellAgainstADropOff() {
    FMSRouter laneRouter = routerOverThePickupLane();
    TransportOrder approach = legOrder("TO-early", LANE_MID, EARLIER, RetreatHolds.APPROACH_LEG)
        .withState(TransportOrder.State.BEING_PROCESSED)
        .withProcessingVehicle(new Vehicle("V2").getReference());
    TransportOrder dropoff = legOrder("TO-late", LANE_MID, LATER, RetreatHolds.DROPOFF_LEG)
        .withIntendedVehicle(new Vehicle("V1").getReference());
    liveOrders(approach, dropoff);

    assertThat(laneRouter.checkGeneralRoutability(dropoff))
        .as("the hand-over is between the two legs of ONE vehicle; two vehicles aimed at the same"
            + " cell are still two bodies")
        .isFalse();
    verify(objectService).updateObjectProperty(
        eq(dropoff.getReference()), eq(FMSRouter.PROPKEY_UNROUTABLE_REASON),
        eq("duplicateGoal:0516:TO-early"));
  }

  @Test
  void anApproachOrderBypassesGoalClaimChecks() {
    FMSRouter laneRouter = routerOverThePickupLane();
    TransportOrder held = legOrder("TO-early", LANE_MID, EARLIER, RetreatHolds.APPROACH_LEG)
        .withState(TransportOrder.State.BEING_PROCESSED)
        .withProcessingVehicle(new Vehicle("V1").getReference());
    TransportOrder second = legOrder("TO-late", LANE_MID, LATER, RetreatHolds.APPROACH_LEG)
        .withIntendedVehicle(new Vehicle("V1").getReference());
    liveOrders(held, second);

    assertThat(laneRouter.checkGeneralRoutability(second))
        .isTrue();
    verify(delegate).checkGeneralRoutability(second);
    verify(objectService, never()).updateObjectProperty(
        eq(second.getReference()), eq(FMSRouter.PROPKEY_UNROUTABLE_REASON), any());
  }

  @Test
  void anUntaggedOrderBypassesGoalClaimChecks() {
    FMSRouter laneRouter = routerOverThePickupLane();
    TransportOrder approach = legOrder("TO-early", LANE_MID, EARLIER, RetreatHolds.APPROACH_LEG)
        .withState(TransportOrder.State.BEING_PROCESSED)
        .withProcessingVehicle(new Vehicle("V1").getReference());
    TransportOrder untagged = orderAimedAt("TO-late", LANE_MID, LATER)
        .withIntendedVehicle(new Vehicle("V1").getReference());
    liveOrders(approach, untagged);

    assertThat(laneRouter.checkGeneralRoutability(untagged))
        .isTrue();
    verify(delegate).checkGeneralRoutability(untagged);
    verify(objectService, never()).updateObjectProperty(
        eq(untagged.getReference()), eq(FMSRouter.PROPKEY_UNROUTABLE_REASON), any());
  }

  private FMSRouter routerOverThePickupLane() {
    PlantModel laneModel = new PlantModel("lane")
        .withPoints(Set.of(
            LANE_TOP, LANE_MID, LANE_BOTTOM, SIDE_EXIT,
            CROSSROAD, CROSSROAD_BELOW, CROSSROAD_WEST))
        .withPaths(Set.of(
            oneWay("0511 --- 0516", LANE_TOP, LANE_MID, 750),
            reversible("0516 --- 0521", LANE_MID, LANE_BOTTOM, 750),
            oneWay("0516 --- 0515", LANE_MID, SIDE_EXIT, 950),
            oneWay("0007 --- 0002", CROSSROAD_BELOW, CROSSROAD, 750),
            oneWay("0001 --- 0007", CROSSROAD_WEST, CROSSROAD_BELOW, 950)));

    PlantModelService plantModelService = mock(PlantModelService.class);
    when(plantModelService.getPlantModel()).thenReturn(laneModel);
    when(delegate.checkGeneralRoutability(any())).thenReturn(true);

    FMSRouter built = new FMSRouter(
        delegate, objectService, new MapfCoordinator(plantModelService, new AdgService()));
    built.initialize();
    return built;
  }

  private void liveOrders(TransportOrder... orders) {
    List<TransportOrder> pool = new ArrayList<>(Arrays.asList(orders));
    when(objectService.stream(TransportOrder.class)).thenAnswer(invocation -> pool.stream());
  }

  private static TransportOrder orderAimedAt(String name, Point goal, Instant createdAt) {
    return new TransportOrder(
        name, List.of(new DriveOrder("d0", new DriveOrder.Destination(goal.getReference()))))
        .withCreationTime(createdAt);
  }

  private static TransportOrder dropOffOrderAimedAt(
      String name, Point goal, Instant createdAt) {
    return legOrder(name, goal, createdAt, RetreatHolds.DROPOFF_LEG);
  }

  private static TransportOrder legOrder(
      String name, Point goal, Instant createdAt, String leg) {
    return orderAimedAt(name, goal, createdAt).withProperty(RetreatHolds.LEG_PROPERTY, leg);
  }

  private static Point point(String name, long x) {
    return new Point(name).withPose(new Pose(new Triple(x, 0, 0), 0.0));
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

  private static Path oneWay(String name, Point src, Point dst, long length) {
    return new Path(name, src.getReference(), dst.getReference())
        .withLength(length)
        .withMaxVelocity(1000)
        .withMaxReverseVelocity(0);
  }

  private static Path reversible(String name, Point src, Point dst, long length) {
    return new Path(name, src.getReference(), dst.getReference())
        .withLength(length)
        .withMaxVelocity(1000)
        .withMaxReverseVelocity(1000);
  }
}
