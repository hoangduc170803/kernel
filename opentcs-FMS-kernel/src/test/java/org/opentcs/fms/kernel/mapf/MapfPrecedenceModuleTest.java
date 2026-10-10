package org.opentcs.fms.kernel.mapf;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.components.kernel.Scheduler;
import org.opentcs.components.kernel.services.InternalPlantModelService;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.TCSResource;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.fms.kernel.mapf.adg.Adg;
import org.opentcs.fms.kernel.mapf.adg.AdgService;
import org.opentcs.fms.kernel.mapf.adg.Step;

class MapfPrecedenceModuleTest {

  private static final Map<String, List<String>> HANDOVER = Map.of(
      "V1", List.of("B", "C"),
      "V2", List.of("A", "B"));

  private static final Map<String, List<String>> PADDED = Map.of(
      "V10", List.of("0088", "0106", "0088", "0087", "0086"));

  private static final Map<String, List<String>> REVISITING = Map.of(
      "V6", List.of("0087", "0088", "0089", "0090", "0091", "0090", "0108"),
      "V7", List.of("0107", "0089", "0005"));

  private static final Map<String, List<String>> DROP_OFF_IN_DEAD_END = Map.of(
      "V1", List.of("0510", "0515"),
      "V8", List.of("0449", "0385", "0531", "0532"));

  private static final Map<String, List<String>> RETREAT_OUT_OF_DEAD_END = Map.of(
      "V1", List.of("0515", "0510", "0531"),
      "V8", List.of("0385", "0531", "0532"));

  private final Map<String, String> positions = new HashMap<>();
  private final Set<String> withoutOpenTcsOrder = new HashSet<>();
  private final AtomicReference<Adg> graph = new AtomicReference<>(Adg.EMPTY);
  private MapfPrecedenceModule module;

  @BeforeEach
  void setUp() {
    AdgService adgService = mock(AdgService.class);
    when(adgService.adg()).thenAnswer(invocation -> graph.get());
    InternalPlantModelService plantModelService = mock(InternalPlantModelService.class);
    when(plantModelService.fetch(eq(Vehicle.class), anyString()))
        .thenAnswer(invocation -> Optional.of(vehicle(invocation.getArgument(1))));
    when(plantModelService.fetch(eq(Vehicle.class))).thenAnswer(invocation -> located());
    module = new MapfPrecedenceModule(adgService, plantModelService);
    module.initialize();
  }

  @Test
  void failsOpenWhenThereIsNoGraph() {
    at("V2", "A");

    assertThat(mayAllocate("V2", "B")).isTrue();
  }

  @Test
  void aVehicleWithNoBlockerMayEnterItsNextStep() {
    install(Map.of(new Step("V2", 1), List.of(new Step("V1", 0))), HANDOVER, 1L);
    at("V1", "B");
    at("V2", "A");
    claiming("V1", "C");

    assertThat(mayAllocate("V1", "C")).isTrue();
  }

  @Test
  void theWaiterIsHeldUntilTheBlockerLeavesItsStep() {
    install(Map.of(new Step("V2", 1), List.of(new Step("V1", 0))), HANDOVER, 1L);
    at("V1", "B");
    at("V2", "A");
    claiming("V2", "B");

    assertThat(mayAllocate("V2", "B")).isFalse();

    at("V1", "C");
    assertThat(mayAllocate("V2", "B")).isTrue();
  }

  @Test
  void aPointThatIsNotTheVehiclesNextStepIsNotConstrained() {
    install(Map.of(new Step("V2", 1), List.of(new Step("V1", 0))), HANDOVER, 1L);
    at("V1", "B");
    at("V2", "A");
    claiming("V2", "B");

    assertThat(mayAllocate("V2", "Z")).isTrue();
  }

  @Test
  void aBlockerWithoutAnOpenTcsOrderIsNotWaitedFor() {
    install(Map.of(new Step("V2", 1), List.of(new Step("V1", 0))), HANDOVER, 1L);
    at("V1", "B");
    at("V2", "A");
    claiming("V2", "B");
    withoutOpenTcsOrder.add("V1");

    assertThat(mayAllocate("V2", "B")).isTrue();
  }

  @Test
  void aVehicleIsNeverHeldByAnEdgeNamingItself() {
    install(Map.of(new Step("V2", 1), List.of(new Step("V2", 0))), HANDOVER, 1L);
    at("V2", "A");
    claiming("V2", "B");

    assertThat(mayAllocate("V2", "B")).isTrue();
  }

  @Test
  void askingTwiceWhileStandingStillGivesTheSameAnswer() {
    install(Map.of(new Step("V2", 1), List.of(new Step("V1", 0))), HANDOVER, 1L);
    at("V1", "B");
    at("V2", "A");
    claiming("V2", "B");

    assertThat(mayAllocate("V2", "B")).isFalse();
    assertThat(mayAllocate("V2", "B")).isFalse();
    assertThat(mayAllocate("V2", "B")).isFalse();
  }

  @Test
  void theStepUnderRequestComesFromWhatIsLeftToAllocateNotFromWhereTheVehicleStands() {
    install(Map.of(new Step("V6", 6), List.of(new Step("V7", 1))), REVISITING, 1L);
    at("V7", "0089");
    at("V6", "0090");
    claiming("V6", "0108");

    assertThat(mayAllocate("V6", "0108")).isFalse();
  }

  @Test
  void theSameCellVisitedTwiceResolvesToWhicheverStepIsStillToBeAllocated() {
    install(
        Map.of(
            new Step("V6", 3), List.of(new Step("V7", 1)),
            new Step("V6", 5), List.of(new Step("V7", 0))),
        REVISITING, 1L);
    at("V7", "0089");
    at("V6", "0091");

    claiming("V6", "0090", "0108");
    assertThat(mayAllocate("V6", "0090")).isTrue();

    claiming("V6", "0090", "0091", "0090", "0108");
    assertThat(mayAllocate("V6", "0090")).isFalse();
  }

  @Test
  void thePointsAppendedByTheGoalPaddingAreNeverAllocated() {
    installPadded(1);
    at("V10", "0088");

    claiming("V10", "0106", "0088", "0087", "0086");
    assertThat(mayAllocate("V10", "0106")).isTrue();

    claiming("V10", "0088", "0087", "0086");
    assertThat(mayAllocate("V10", "0088")).isFalse();
  }

  @Test
  void aPointOutsideTheProvenRouteIsHeldEvenWhenTheClaimExtendsPastTheAdgRoute() {
    graph.set(Adg.of(
        Map.of(), Map.of("V6", List.of("0137", "0147")), Map.of("V6", 0))
        .stamped(1L, System.currentTimeMillis()));
    at("V6", "0137");
    claiming("V6", "0147", "0157", "0167", "0157", "0147");

    assertThat(mayAllocate("V6", "0147")).isFalse();
  }

  @Test
  void aPointOutsideTheProvenRouteIsHeldWhenTheAdgContainsOnlyTheCurrentPoint() {
    graph.set(Adg.of(
        Map.of(), Map.of("V6", List.of("0137")), Map.of("V6", 0))
        .stamped(1L, System.currentTimeMillis()));
    at("V6", "0137");
    claiming("V6", "0147", "0157", "0167");

    assertThat(mayAllocate("V6", "0147")).isFalse();
  }

  @Test
  void anOldPlanStillHoldsAPointThatWasNeverProven() {
    graph.set(Adg.of(
        Map.of(), Map.of("V6", List.of("0137", "0147")), Map.of("V6", 0))
        .stamped(1L, System.currentTimeMillis() - 60_000L));
    at("V6", "0137");
    claiming("V6", "0147", "0157", "0167");

    assertThat(mayAllocate("V6", "0147")).isFalse();
  }

  @Test
  void aVehicleHeldAtTheHorizonStaysHeldWhileTheSamePlanStands() {
    installPadded(1);
    at("V10", "0106");
    claiming("V10", "0088", "0087", "0086");

    assertThat(mayAllocate("V10", "0088")).isFalse();
    assertThat(mayAllocate("V10", "0088")).isFalse();
  }

  @Test
  void aPlanWhoseTailWasNeverAppendedConstrainsNothing() {
    graph.set(Adg.of(Map.of(), PADDED, Map.of("V10", 4))
        .stamped(1L, System.currentTimeMillis()));
    at("V10", "0088");
    claiming("V10", "0088", "0087", "0086");

    assertThat(mayAllocate("V10", "0088")).isTrue();
  }

  @Test
  void aPlanWithNoHorizonRecordedFailsOpen() {
    install(Map.of(), PADDED, 1L);
    at("V10", "0088");
    claiming("V10", "0088", "0087", "0086");

    assertThat(mayAllocate("V10", "0088")).isTrue();
  }

  @Test
  void aPlanTooOldToTrustFailsOpen() {
    graph.set(Adg.of(Map.of(), PADDED, Map.of("V10", 1))
        .stamped(1L, System.currentTimeMillis() - 60_000L));
    at("V10", "0088");
    claiming("V10", "0088", "0087", "0086");

    assertThat(mayAllocate("V10", "0088")).isTrue();
  }

  @Test
  void aDriveOrderThePlanNeverCoveredIsHeldUntilASolveCoversIt() {
    install(Map.of(new Step("V8", 2), List.of(new Step("V1", 0))), DROP_OFF_IN_DEAD_END, 1L);
    at("V1", "0515");
    at("V8", "0449");
    claiming("V1", "0510", "0531");

    assertThat(mayAllocate("V1", "0510")).isFalse();

    install(
        Map.of(new Step("V1", 2), List.of(new Step("V8", 1))), RETREAT_OUT_OF_DEAD_END, 2L);
    at("V8", "0385");

    assertThat(mayAllocate("V1", "0510")).isTrue();
  }

  @Test
  void futureDriveOrdersInTheClaimDoNotHideTheStepUnderRequest() {
    install(Map.of(new Step("V8", 2), List.of(new Step("V1", 0))), DROP_OFF_IN_DEAD_END, 1L);
    at("V1", "0515");
    at("V8", "0449");
    claiming("V1", "0515", "0510", "0531");

    assertThat(mayAllocate("V1", "0515")).isTrue();
  }

  @Test
  void aPointTheVehicleHasAlreadyPassedIsHeld() {
    install(Map.of(new Step("V8", 2), List.of(new Step("V1", 0))), DROP_OFF_IN_DEAD_END, 1L);
    at("V1", "0515");
    at("V8", "0449");
    claiming("V1", "0515", "0510");

    assertThat(mayAllocate("V1", "0510")).isFalse();
  }

  private void installPadded(int coordinatedStep) {
    graph.set(Adg.of(Map.of(), PADDED, Map.of("V10", coordinatedStep))
        .stamped(1L, System.currentTimeMillis()));
  }

  private void install(
      Map<Step, List<Step>> dependencies, Map<String, List<String>> routes, long generation) {
    graph.set(Adg.of(dependencies, routes).stamped(generation, System.currentTimeMillis()));
  }

  private void at(String vehicle, String point) {
    positions.put(vehicle, point);
  }

  private void claiming(String vehicle, String... stillToAllocate) {
    List<Set<TCSResource<?>>> remaining = new ArrayList<>();
    for (String point : stillToAllocate) {
      remaining.add(Set.<TCSResource<?>>of(new Point(point)));
    }
    module.setAllocationState(client(vehicle), Set.of(), remaining);
  }

  private boolean mayAllocate(String vehicle, String point) {
    return module.mayAllocate(client(vehicle), Set.<TCSResource<?>>of(new Point(point)));
  }

  private Set<Vehicle> located() {
    Set<Vehicle> fleet = new HashSet<>();
    positions.forEach((name, point) -> fleet.add(
        vehicle(name).withCurrentPosition(new Point(point).getReference())));
    return fleet;
  }

  private Vehicle vehicle(String name) {
    Vehicle candidate = new Vehicle(name);
    return withoutOpenTcsOrder.contains(name)
        ? candidate
        : candidate.withTransportOrder(
            new TransportOrder("order-" + name, List.of()).getReference());
  }

  private static Scheduler.Client client(String vehicle) {
    Scheduler.Client stub = mock(Scheduler.Client.class);
    when(stub.getRelatedVehicle()).thenReturn(new Vehicle(vehicle).getReference());
    return stub;
  }
}
