package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;

import org.junit.jupiter.api.Test;

class PathUtilsTest {

  @Test
  void compressRemovesWaitsButKeepsRevisitedPoints() {
    assertThat(PathUtils.compress(List.of("A", "A", "B", "B", "A")))
        .containsExactly("A", "B", "A");
  }

  @Test
  void copyOfDefensivelyCopiesAndPadsAgentPaths() {
    int[][] paths = { { 1, 2 }, null };

    int[][] copy = PathUtils.copyOf(paths, 3);
    paths[0][0] = 9;

    assertThat(copy.length).isEqualTo(3);
    assertThat(copy[0]).containsExactly(1, 2);
    assertThat(copy[1]).isEmpty();
    assertThat(copy[2]).isEmpty();
  }

  // --- compress(int[]): the Rule #1 pure-wait stripper (excursions kept) ------

  @Test
  void stripsWaits() {
    assertThat(PathUtils.compress(new int[] { 1, 1, 2, 2, 3 })).containsExactly(1, 2, 3);
  }

  @Test
  void keepsExcursionBackToStart() {
    // 1->2->1->3: the dip to 2 IS the plan's yield manoeuvre. Since 2026-07-08 it
    // is KEPT in the driven route -- openTCS supports revisited points (verified:
    // Route has no such restriction; ReservationEntry counts re-allocations; the
    // return leg is the same Path driven BACKWARD). Only waits are stripped.
    assertThat(PathUtils.compress(new int[] { 1, 2, 1, 3 })).containsExactly(1, 2, 1, 3);
  }

  @Test
  void keepsExcursionMidPathButStripsWaits() {
    // 1->2->3->3->2->4: the wait at 3 collapses; the dip-and-return stays.
    assertThat(PathUtils.compress(new int[] { 1, 2, 3, 3, 2, 4 }))
        .containsExactly(1, 2, 3, 2, 4);
  }

  @Test
  void keepsCleanForwardPath() {
    assertThat(PathUtils.compress(new int[] { 1, 2, 3, 4 })).containsExactly(1, 2, 3, 4);
  }

  @Test
  void singletonStaysPut() {
    assertThat(PathUtils.compress(new int[] { 5 })).containsExactly(5);
  }
}
