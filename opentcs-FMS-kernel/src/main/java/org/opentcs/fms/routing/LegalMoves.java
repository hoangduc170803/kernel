package org.opentcs.fms.routing;

import java.util.Arrays;

public final class LegalMoves {

  private static final int MOTION_STATES = Roadmap.NOSES + 1;

  private final Roadmap roadmap;
  private final RetreatRule retreatRule;
  private final int[] retreatGoals;
  private final int[] keyByAgent;
  private final int[] retreatGoalByKey;
  private final int[][][] candidatesByKey;
  private final int[][] originalLengthsByKey;
  private final int[][] distancesByKey;
  private final boolean[] distancesResolvedByKey;
  private int filteredCandidates;
  private int forcingStay;

  public LegalMoves(Roadmap roadmap) {
    this(roadmap, null);
  }

  public LegalMoves(Roadmap roadmap, int[] retreatGoals) {
    this.roadmap = roadmap;
    this.retreatGoals = retreatGoals == null ? new int[0] : retreatGoals.clone();
    this.retreatRule = new RetreatRule();
    int agentCount = retreatGoals == null ? 0 : retreatGoals.length;
    this.keyByAgent = new int[agentCount];
    int[] keyRetreatGoals = new int[agentCount + 1];
    keyRetreatGoals[0] = -1;
    int keyCount = 1;
    for (int agent = 0; agent < agentCount; agent++) {
      int goal = retreatGoals == null || agent >= retreatGoals.length ? -1 : retreatGoals[agent];
      int key = 0;
      while (key < keyCount && keyRetreatGoals[key] != goal) {
        key++;
      }
      if (key == keyCount) {
        keyRetreatGoals[keyCount] = goal;
        keyCount++;
      }
      keyByAgent[agent] = key;
    }
    this.retreatGoalByKey = Arrays.copyOf(keyRetreatGoals, keyCount);
    int slotCount = roadmap.size() * MOTION_STATES;
    this.candidatesByKey = new int[keyCount][slotCount][];
    this.originalLengthsByKey = new int[keyCount][slotCount];
    this.distancesByKey = new int[keyCount][];
    this.distancesResolvedByKey = new boolean[keyCount];
  }

  public int[] forAgent(int agent, int vertex, int motion) {
    if (agent < 0 || agent >= keyByAgent.length
        || motion < Roadmap.DIR_NONE || motion >= Roadmap.NOSES) {
      int slot = slotOf(vertex, motion);
      if (slot < 0 || slot >= candidatesByKey[0].length) {
        return roadmap.kinematicCandidates(vertex, motion);
      }
      int canonicalVertex = slot / MOTION_STATES;
      int canonicalMotion = slot % MOTION_STATES - 1;
      return candidatesForKey(0, slot, canonicalVertex, canonicalMotion, vertex);
    }
    int slot = slotOf(vertex, motion);
    return candidatesForKey(keyByAgent[agent], slot, vertex, motion, vertex);
  }

  public int filteredCandidates() {
    return filteredCandidates;
  }

  public int forcingStay() {
    return forcingStay;
  }

  public int[] retreatGoals() {
    return retreatGoals.clone();
  }

  private int[] candidatesForKey(
      int key, int slot, int vertex, int motion, int onlyStayVertex) {
    int[] legal = candidatesByKey[key][slot];
    if (legal == null) {
      int[] original = roadmap.kinematicCandidates(vertex, motion);
      int[] distances = distancesForKey(key);
      legal = retreatRule.filterCandidates(
          original, vertex, retreatGoalByKey[key], motion, distances);
      candidatesByKey[key][slot] = legal;
      originalLengthsByKey[key][slot] = original.length;
    }
    int removed = originalLengthsByKey[key][slot] - legal.length;
    filteredCandidates += removed;
    if (removed > 0 && onlyStays(legal, onlyStayVertex)) {
      forcingStay++;
    }
    return legal.clone();
  }

  private int[] distancesForKey(int key) {
    if (!distancesResolvedByKey[key]) {
      int goal = retreatGoalByKey[key];
      distancesByKey[key] = goal >= 0 && goal < roadmap.size()
          ? roadmap.distanceTo(goal)
          : null;
      distancesResolvedByKey[key] = true;
    }
    return distancesByKey[key];
  }

  private static boolean onlyStays(int[] candidates, int vertex) {
    return candidates.length == 1 && candidates[0] == vertex;
  }

  private static int slotOf(int vertex, int motion) {
    return vertex * MOTION_STATES + motion + 1;
  }
}
