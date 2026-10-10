package org.opentcs.fms.kernel.mapf;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.opentcs.components.kernel.services.InternalTCSObjectService;
import org.opentcs.data.model.Location;
import org.opentcs.data.model.LocationType;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Pose;
import org.opentcs.data.model.Triple;
import org.opentcs.data.order.DriveOrder;

class GoalResolverTest {

  private static Point point(String name) {
    return new Point(name).withPose(new Pose(new Triple(0, 0, 0), 0.0));
  }

  @Test
  void resolvesPointDestinationDirectly() {
    Point p = point("P3");
    DriveOrder.Destination dest = new DriveOrder.Destination(p.getReference());
    InternalTCSObjectService service = mock(InternalTCSObjectService.class);

    assertThat(GoalResolver.goalPoint(dest, service)).contains("P3");
  }

  @Test
  void resolvesSinglePointLocationToItsPoint() {
    LocationType type = new LocationType("lt");
    Point p7 = point("P7");
    Location bare = new Location("Loc1", type.getReference());
    Location loc = bare.withAttachedLinks(
        Set.of(new Location.Link(bare.getReference(), p7.getReference())));

    InternalTCSObjectService service = mock(InternalTCSObjectService.class);
    when(service.fetch(Location.class, "Loc1")).thenReturn(Optional.of(loc));

    DriveOrder.Destination dest = new DriveOrder.Destination(loc.getReference());
    assertThat(GoalResolver.goalPoint(dest, service)).contains("P7");
  }

  @Test
  void emptyWhenLocationNotFound() {
    LocationType type = new LocationType("lt");
    Location loc = new Location("Ghost", type.getReference());
    InternalTCSObjectService service = mock(InternalTCSObjectService.class);
    when(service.fetch(Location.class, "Ghost")).thenReturn(Optional.empty());

    DriveOrder.Destination dest = new DriveOrder.Destination(loc.getReference());
    assertThat(GoalResolver.goalPoint(dest, service)).isEmpty();
  }

  @Test
  void candidatesReturnAllLinkedPointsSortedByName() {
    LocationType type = new LocationType("lt");
    Point p7 = point("P7");
    Point p2 = point("P2");
    Location bare = new Location("Loc1", type.getReference());
    Location loc = bare.withAttachedLinks(Set.of(
        new Location.Link(bare.getReference(), p7.getReference()),
        new Location.Link(bare.getReference(), p2.getReference())));

    InternalTCSObjectService service = mock(InternalTCSObjectService.class);
    when(service.fetch(Location.class, "Loc1")).thenReturn(Optional.of(loc));

    DriveOrder.Destination dest = new DriveOrder.Destination(loc.getReference());
    assertThat(GoalResolver.goalCandidates(dest, service))
        .containsExactly("P2", "P7"); // both bays, deterministic (name) order
  }

  @Test
  void goalPointResolvesToSinglePointEvenIfLocationHasMultipleLinks() {
    LocationType type = new LocationType("lt");
    Point p7 = point("P7");
    Point p2 = point("P2");
    Location bare = new Location("Loc1", type.getReference());
    Location loc = bare.withAttachedLinks(Set.of(
        new Location.Link(bare.getReference(), p7.getReference()),
        new Location.Link(bare.getReference(), p2.getReference())));

    InternalTCSObjectService service = mock(InternalTCSObjectService.class);
    when(service.fetch(Location.class, "Loc1")).thenReturn(Optional.of(loc));

    DriveOrder.Destination dest = new DriveOrder.Destination(loc.getReference());
    assertThat(GoalResolver.goalPoint(dest, service)).contains("P2");
  }

  @Test
  void candidatesKeepOnlyLinksAllowingTheOperation() {
    LocationType type = new LocationType("lt");
    Point pLoad = point("P1");
    Point pAny = point("P2");
    Point pUnload = point("P3");
    Location bare = new Location("Loc1", type.getReference());
    Location loc = bare.withAttachedLinks(Set.of(
        new Location.Link(bare.getReference(), pLoad.getReference())
            .withAllowedOperations(Set.of("LOAD")),
        new Location.Link(bare.getReference(), pAny.getReference()), // empty = any op
        new Location.Link(bare.getReference(), pUnload.getReference())
            .withAllowedOperations(Set.of("UNLOAD"))));

    InternalTCSObjectService service = mock(InternalTCSObjectService.class);
    when(service.fetch(Location.class, "Loc1")).thenReturn(Optional.of(loc));

    DriveOrder.Destination dest =
        new DriveOrder.Destination(loc.getReference()).withOperation("LOAD");
    assertThat(GoalResolver.goalCandidates(dest, service))
        .isEqualTo(List.of("P1", "P2")); // P3 excluded (UNLOAD only)
  }
}
