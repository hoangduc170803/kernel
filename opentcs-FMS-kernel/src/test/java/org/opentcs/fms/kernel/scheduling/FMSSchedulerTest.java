package org.opentcs.fms.kernel.scheduling;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatCode;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import java.util.List;
import java.util.Set;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ScheduledExecutorService;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.components.kernel.Scheduler;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.TCSResource;
import org.opentcs.strategies.basic.scheduling.AllocationAdvisor;
import org.opentcs.strategies.basic.scheduling.DefaultScheduler;
import org.opentcs.strategies.basic.scheduling.ReservationPool;
import org.opentcs.util.event.EventBus;

class FMSSchedulerTest {

  private static final Set<TCSResource<?>> A = Set.of(new Point("A"));
  private static final Set<TCSResource<?>> B = Set.of(new Point("B"));
  private static final Set<TCSResource<?>> C = Set.of(new Point("C"));
  private static final Set<TCSResource<?>> OFF_ROUTE = Set.of(new Point("Z"));

  private ReservationPool reservationPool;
  private Scheduler.Client client;
  private FMSScheduler scheduler;
  private DefaultScheduler stockScheduler;
  private AllocationAdvisor allocationAdvisor;
  private ScheduledExecutorService kernelExecutor;

  @BeforeEach
  void setUp() {
    reservationPool = new ReservationPool();
    client = mock(Scheduler.Client.class);
    when(client.getId()).thenReturn("Vehicle-0002");

    allocationAdvisor = mock(AllocationAdvisor.class);
    kernelExecutor = mock(ScheduledExecutorService.class);
    EventBus eventBus = mock(EventBus.class);
    Object globalSyncObject = new Object();

    scheduler = new FMSScheduler(
        allocationAdvisor, reservationPool, kernelExecutor, eventBus, globalSyncObject
    );
    stockScheduler = new DefaultScheduler(
        allocationAdvisor, reservationPool, kernelExecutor, eventBus, globalSyncObject
    );
  }

  @Test
  void anAllocationAtTheClaimHeadIsPassedStraightThrough() {
    scheduler.claim(client, List.of(A, B, C));

    assertThatCode(() -> scheduler.allocate(client, A)).doesNotThrowAnyException();

    assertThat(reservationPool.getClaim(client))
        .as("an in-sync claim must be left untouched")
        .containsExactly(A, B, C);
  }

  @Test
  void aStaleClaimKillsTheVehicleUnderTheStockScheduler() {
    stockScheduler.claim(client, List.of(A, B, C));

    assertThatThrownBy(() -> stockScheduler.allocate(client, B))
        .as("this is the exception that kills the command pump and zombifies the vehicle")
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessageContaining("Not the next claimed resources");
  }

  @Test
  void aClaimThatLagsTheRequestIsResyncedWithoutLosingTheLookahead() {
    scheduler.claim(client, List.of(A, B, C));

    assertThatCode(() -> scheduler.allocate(client, B)).doesNotThrowAnyException();

    assertThat(reservationPool.getClaim(client))
        .as("the consumed prefix is dropped but the tail must survive: a single-entry claim would"
            + " be emptied by the unclaim on grant, so every later step would mismatch again")
        .containsExactly(B, C);
  }

  @Test
  void aRequestOffTheClaimedRouteIsClaimedOnItsOwn() {
    scheduler.claim(client, List.of(A, B, C));

    assertThatCode(() -> scheduler.allocate(client, OFF_ROUTE)).doesNotThrowAnyException();

    assertThat(reservationPool.getClaim(client)).containsExactly(OFF_ROUTE);
  }

  @Test
  void anAllocationWithoutAnyClaimIsRecoveredRatherThanThrown() {
    assertThatCode(() -> scheduler.allocate(client, A)).doesNotThrowAnyException();

    assertThat(reservationPool.getClaim(client)).containsExactly(A);
  }

  @Test
  void aClaimDiscardsTheClientsDeferredAllocations() {
    runAllocatorTasksInline();
    scheduler.claim(client, List.of(A, B, C));
    scheduler.allocate(client, A);

    scheduler.claim(client, List.of(A, B, C));
    scheduler.reschedule();

    verify(allocationAdvisor, times(1)).mayAllocate(client, A);
  }

  @Test
  void theStockSchedulerReplaysADeferredAllocationForeverAcrossReClaims() {
    runAllocatorTasksInline();
    stockScheduler.claim(client, List.of(A, B, C));
    stockScheduler.allocate(client, A);

    stockScheduler.claim(client, List.of(A, B, C));
    stockScheduler.reschedule();

    verify(allocationAdvisor, times(2)).mayAllocate(client, A);
  }

  @Test
  void aClaimLeavesAnotherClientsDeferredAllocationsAlone() {
    runAllocatorTasksInline();
    Scheduler.Client other = mock(Scheduler.Client.class);
    when(other.getId()).thenReturn("Vehicle-0003");
    scheduler.claim(other, List.of(B));
    scheduler.allocate(other, B);

    scheduler.claim(client, List.of(A));
    scheduler.reschedule();

    verify(allocationAdvisor, times(2))
        .mayAllocate(other, B);
  }

  private void runAllocatorTasksInline() {
    when(kernelExecutor.submit(any(Runnable.class))).thenAnswer(invocation -> {
      invocation.getArgument(0, Runnable.class).run();
      return CompletableFuture.completedFuture(null);
    });
  }
}
