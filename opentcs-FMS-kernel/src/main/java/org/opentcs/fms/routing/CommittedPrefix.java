package org.opentcs.fms.routing;

import java.util.Arrays;

final class CommittedPrefix {

  private static final CommittedPrefix NONE = new CommittedPrefix(new int[0]);

  private final int[] cells;

  private CommittedPrefix(int[] cells) {
    this.cells = cells;
  }

  static CommittedPrefix of(int[] cells) {
    return cells == null || cells.length == 0
        ? NONE
        : new CommittedPrefix(cells.clone());
  }

  static CommittedPrefix[] copyOf(int[][] prefixes, int agents) {
    CommittedPrefix[] copy = new CommittedPrefix[agents];
    for (int agent = 0; agent < agents; agent++) {
      copy[agent] = of(prefixes == null || agent >= prefixes.length ? null : prefixes[agent]);
    }
    return copy;
  }

  int length() {
    return cells.length;
  }

  boolean isEmpty() {
    return cells.length == 0;
  }

  boolean isActiveAt(int depth, int cell) {
    return depth >= 0 && depth < cells.length - 1 && cells[depth] == cell;
  }

  int committedMoveAt(int depth) {
    return cells[depth + 1];
  }

  CommittedPrefix remainingFrom(int depth, int maxLength) {
    if (depth < 0 || depth >= cells.length || maxLength <= 0) {
      return NONE;
    }
    int length = Math.min(maxLength, cells.length - depth);
    return new CommittedPrefix(Arrays.copyOfRange(cells, depth, depth + length));
  }

  int frontierTick() {
    return cells.length - 1;
  }

  int frontier() {
    return cells[cells.length - 1];
  }

  void copyInto(int[] path) {
    Arrays.fill(path, CollisionTable.NO_VERTEX);
    System.arraycopy(cells, 0, path, 0, cells.length);
  }

  int[] guidePathOver(int[] warmStart, int window) {
    int[] path = new int[window];
    if (!matchesWarmStart(warmStart)) {
      copyInto(path);
      return path;
    }

    int copied = Math.min(window, warmStart.length);
    System.arraycopy(warmStart, 0, path, 0, copied);
    if (copied < window) {
      int lastCell = copied == 0 ? CollisionTable.NO_VERTEX : warmStart[copied - 1];
      Arrays.fill(path, copied, window, lastCell);
    }
    return path;
  }

  private boolean matchesWarmStart(int[] warmStart) {
    if (warmStart == null || warmStart.length < cells.length) {
      return false;
    }
    for (int tick = 0; tick < cells.length; tick++) {
      if (warmStart[tick] != cells[tick]) {
        return false;
      }
    }
    return true;
  }

  int noseAfter(Roadmap roadmap, int startNose) {
    int nose = startNose;
    for (int step = 1; step < cells.length; step++) {
      nose = roadmap.noseAfter(nose, cells[step - 1], cells[step]);
    }
    return nose;
  }

  int motionAfter(Roadmap roadmap, int fallbackMotion) {
    return cells.length < 2
        ? fallbackMotion
        : roadmap.motionAfter(cells[cells.length - 2], cells[cells.length - 1]);
  }

  boolean isViolatedAt(int depth, int cellAtDepth, int cellAtNextDepth) {
    return cells[depth] == cellAtDepth && cells[depth + 1] != cellAtNextDepth;
  }
}
