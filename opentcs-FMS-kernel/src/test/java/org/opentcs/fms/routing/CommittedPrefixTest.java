package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;

class CommittedPrefixTest {

  @Test
  void ofCopiesCellsAndUsesAnEmptyValueForNullOrEmptyInput() {
    int[] cells = { 4, 5 };
    CommittedPrefix prefix = CommittedPrefix.of(cells);
    cells[1] = 9;

    assertThat(prefix.length()).isEqualTo(2);
    assertThat(prefix.isEmpty()).isFalse();
    assertThat(prefix.frontier()).isEqualTo(5);
    assertThat(CommittedPrefix.of(null).isEmpty()).isTrue();
    assertThat(CommittedPrefix.of(new int[0]).isEmpty()).isTrue();
  }

  @Test
  void copyOfPreservesAgentCountAndEmptyRows() {
    CommittedPrefix[] prefixes = CommittedPrefix.copyOf(new int[][] { null, { 3, 4 } }, 3);

    assertThat(prefixes).hasSize(3);
    assertThat(prefixes[0].isEmpty()).isTrue();
    assertThat(prefixes[1].length()).isEqualTo(2);
    assertThat(prefixes[1].frontier()).isEqualTo(4);
    assertThat(prefixes[2].isEmpty()).isTrue();
    assertThat(CommittedPrefix.copyOf(null, 1)[0].isEmpty()).isTrue();
  }

  @Test
  void isActiveAtRequiresCurrentCellAndACommittedMoveAfterIt() {
    CommittedPrefix prefix = CommittedPrefix.of(new int[] { 7, 8, 9 });

    assertThat(CommittedPrefix.of(null).isActiveAt(0, 7)).isFalse();
    assertThat(CommittedPrefix.of(new int[] { 7 }).isActiveAt(0, 7)).isFalse();
    assertThat(prefix.isActiveAt(-1, 7)).isFalse();
    assertThat(prefix.isActiveAt(0, 6)).isFalse();
    assertThat(prefix.isActiveAt(0, 7)).isTrue();
    assertThat(prefix.isActiveAt(2, 9)).isFalse();
  }

  @Test
  void committedMoveAtReadsTheCellAfterTheCurrentDepth() {
    assertThat(CommittedPrefix.of(new int[] { 7, 8, 9 }).committedMoveAt(1)).isEqualTo(9);
  }

  @Test
  void remainingFromSlicesAtTheRequestedDepthAndCapsTheWindow() {
    CommittedPrefix remaining = CommittedPrefix.of(new int[] { 2, 3, 4, 5 })
        .remainingFrom(1, 2);
    int[] path = new int[4];
    remaining.copyInto(path);

    assertThat(remaining.length()).isEqualTo(2);
    assertThat(remaining.frontierTick()).isEqualTo(1);
    assertThat(remaining.frontier()).isEqualTo(4);
    assertThat(path).containsExactly(3, 4, CollisionTable.NO_VERTEX, CollisionTable.NO_VERTEX);
    assertThat(remaining.remainingFrom(4, 2).isEmpty()).isTrue();
    assertThat(remaining.remainingFrom(0, 0).isEmpty()).isTrue();
  }

  @Test
  void frontierTickIsTheLastTick() {
    assertThat(CommittedPrefix.of(new int[] { 2, 3, 4 }).frontierTick()).isEqualTo(2);
    assertThat(CommittedPrefix.of(null).frontierTick()).isEqualTo(-1);
  }

  @Test
  void copyIntoClearsThePathThenCopiesThePrefix() {
    CommittedPrefix prefix = CommittedPrefix.of(new int[] { 2, 3 });
    int[] path = { 8, 8, 8 };
    prefix.copyInto(path);
    path[0] = 9;

    assertThat(path).containsExactly(9, 3, CollisionTable.NO_VERTEX);
    assertThat(prefix.isActiveAt(0, 2)).isTrue();

    int[] emptyPath = { 8, 8 };
    CommittedPrefix.of(null).copyInto(emptyPath);
    assertThat(emptyPath).containsExactly(CollisionTable.NO_VERTEX, CollisionTable.NO_VERTEX);
  }

  @Test
  void guidePathOverKeepsMatchingWarmStartTailAndPadsItsLastCell() {
    CommittedPrefix prefix = CommittedPrefix.of(new int[] { 3, 4 });

    assertThat(prefix.guidePathOver(new int[] { 3, 4, 5 }, 5))
        .containsExactly(3, 4, 5, 5, 5);
  }

  @Test
  void guidePathOverFallsBackWhenAnyCommittedTickDiffers() {
    CommittedPrefix prefix = CommittedPrefix.of(new int[] { 3, 4, 5 });

    assertThat(prefix.guidePathOver(new int[] { 3, 9, 5, 7 }, 5))
        .containsExactly(3, 4, 5, CollisionTable.NO_VERTEX, CollisionTable.NO_VERTEX);
  }

  @Test
  void guidePathOverFallsBackWithoutAWarmStart() {
    CommittedPrefix prefix = CommittedPrefix.of(new int[] { 3, 4 });

    assertThat(prefix.guidePathOver(null, 4))
        .containsExactly(3, 4, CollisionTable.NO_VERTEX, CollisionTable.NO_VERTEX);
  }

  @Test
  void guidePathOverFallsBackWhenWarmStartIsShorterThanThePrefix() {
    CommittedPrefix prefix = CommittedPrefix.of(new int[] { 3, 4, 5 });

    assertThat(prefix.guidePathOver(new int[] { 3, 4 }, 5))
        .containsExactly(3, 4, 5, CollisionTable.NO_VERTEX, CollisionTable.NO_VERTEX);
  }

  @Test
  void noseAfterProcessesEveryCommittedMoveAcrossAQuarterTurn() {
    Roadmap roadmap = GridFixture.grid(2, 2);
    CommittedPrefix prefix = CommittedPrefix.of(new int[] {
        GridFixture.id(roadmap, 0, 0),
        GridFixture.id(roadmap, 0, 1),
        GridFixture.id(roadmap, 1, 1)
    });

    assertThat(prefix.noseAfter(roadmap, Roadmap.DIR_NONE)).isEqualTo(Roadmap.DIR_POS_Y);
  }

  @Test
  void motionAfterUsesFallbackForPrefixesShorterThanOneMove() {
    Roadmap roadmap = GridFixture.grid(2, 1);
    int fallback = Roadmap.DIR_NEG_Y;

    assertThat(CommittedPrefix.of(null).motionAfter(roadmap, fallback)).isEqualTo(fallback);
    assertThat(CommittedPrefix.of(new int[] { GridFixture.id(roadmap, 0, 0) })
        .motionAfter(roadmap, fallback)).isEqualTo(fallback);
    assertThat(CommittedPrefix.of(new int[] {
        GridFixture.id(roadmap, 0, 0), GridFixture.id(roadmap, 0, 1)
    }).motionAfter(roadmap, fallback)).isEqualTo(Roadmap.DIR_POS_X);
  }

  @Test
  void isViolatedAtRequiresTheCommittedCurrentCellAndADifferentNextCell() {
    CommittedPrefix prefix = CommittedPrefix.of(new int[] { 2, 3, 4 });

    assertThat(prefix.isViolatedAt(0, 2, 5)).isTrue();
    assertThat(prefix.isViolatedAt(0, 1, 5)).isFalse();
    assertThat(prefix.isViolatedAt(0, 2, 3)).isFalse();
  }
}
