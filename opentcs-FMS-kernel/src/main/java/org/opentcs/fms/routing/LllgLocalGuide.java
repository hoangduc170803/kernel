package org.opentcs.fms.routing;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

import org.opentcs.fms.routing.diag.GuideCounters;
import org.opentcs.fms.routing.pibt.Mulberry32;

/**
 * Faithful vertex-time local guidance from the LLLG reference implementation.
 *
 * <p>In particular, this guide has no heading state, turn/reverse charge, body-footprint
 * reservation, terminal-tail charge, or FMS committed-route prefix. It exists as the comparison
 * profile for {@link FmsLocalGuide}.
 */
public final class LllgLocalGuide implements Guidance {

  private static final int NO_VERTEX = CollisionTable.NO_VERTEX;
  private static final int GUIDE_SEED_MIX = 0x85EBCA6B;
  private static final Comparator<Node> CHEAPEST_THEN_DEEPEST = (left, right) -> {
    if (left.f != right.f) {
      return left.f < right.f ? -1 : 1;
    }
    if (left.g != right.g) {
      return left.g > right.g ? -1 : 1;
    }
    return 0;
  };

  private final boolean on = GuideSettings.enabled();
  private final int window = Math.max(2, Integer.getInteger("fms.lllg.window", 10));
  private final int numRefine = Integer.getInteger("fms.lllg.numRefine", 1);
  private final float collisionCost = Float.parseFloat(
      System.getProperty("fms.lllg.collisionCost", "1.0"));
  private final float collisionCostOrder = Float.parseFloat(
      System.getProperty("fms.lllg.collisionCostOrder", "1e-7"));
  private final boolean deterministic = Boolean.parseBoolean(
      System.getProperty("fms.lllg.deterministic", "false"));
  private final boolean clearGoalFirst = Boolean.parseBoolean(
      System.getProperty("fms.lllg.clearGoalFirst", "false"));

  private final Roadmap roadmap;
  private final DistTable distTable;
  private final int[] goals;
  private final int agents;
  private final Mulberry32 rng;
  private final CollisionTable collisions;
  private final int[][] guidePaths;
  private final Node[][] closed;
  private final List<int[]> closedSlots = new ArrayList<>();
  private final int[] guidedNext;
  private final float scaledCollisionCost;
  private final float scaledCollisionCostOrder;
  private final float[] cachedCollisionCosts;
  private final List<int[][]> guidePathsHistory = new ArrayList<>();
  private final GuideCounters counters = new GuideCounters();

  private int currentStep;

  public LllgLocalGuide(
      Roadmap roadmap, DistTable distTable, int[] goals, int agents, int seed) {
    this.roadmap = roadmap;
    this.distTable = distTable;
    this.goals = goals.clone();
    this.agents = agents;
    this.rng = new Mulberry32(seed ^ GUIDE_SEED_MIX);
    this.collisions = new CollisionTable(roadmap.size(), true, false);
    this.guidePaths = new int[agents][window];
    for (int[] path : guidePaths) {
      Arrays.fill(path, NO_VERTEX);
    }
    this.closed = new Node[window][roadmap.size()];
    this.guidedNext = new int[agents];
    Arrays.fill(guidedNext, NO_VERTEX);
    this.cachedCollisionCosts = new float[agents];
    this.scaledCollisionCost = collisionCost;
    this.scaledCollisionCostOrder = collisionCostOrder;
  }

  @Override
  public boolean enabled() {
    return on;
  }

  @Override
  public int window() {
    return window;
  }

  @Override
  public void construct(int[] configuration, int[] noses, int[] order, int solveDepth) {
    if (!on || numRefine < 0) {
      return;
    }
    boolean[] atGoal = goalFlags(configuration);
    int slid = carryGuidePathsForwardOneStep(configuration);
    if (counters.constructions == 0) {
      counters.warmStartSlides = slid;
    }
    counters.carryForwardSlides += slid;
    counters.constructions++;

    int rounds = numRefine == 0 ? 1 : numRefine;
    for (int round = 0; round < rounds; round++) {
      releaseGoalAgents(atGoal);
      for (int slot = 0; slot < agents; slot++) {
        int agent = order[slot];
        if (atGoal != null && atGoal[agent]) {
          continue;
        }
        if (numRefine == 0 && guidePaths[agent][0] != guidePaths[agent][window - 1]) {
          continue;
        }
        guidedNext[agent] = NO_VERTEX;
        collisions.clearPath(agent, guidePaths[agent]);
        updatePath(agent, configuration);
        collisions.enrollPath(agent, guidePaths[agent]);
      }
      replanGoalAgents(atGoal, configuration);
    }

    for (int agent = 0; agent < agents; agent++) {
      guidedNext[agent] = guidePaths[agent][1];
      collisions.clearPath(agent, guidePaths[agent]);
    }
  }

  @Override
  public int heuristic(int agent, int vertex) {
    if (!on || numRefine < 0 || guidedNext[agent] == NO_VERTEX) {
      return distTable.get(agent, vertex);
    }
    return vertex == guidedNext[agent] ? 0 : distTable.get(agent, vertex) + 1;
  }

  @Override
  public int[] guidedConfiguration() {
    return guidedNext;
  }

  @Override
  public float collisionCostOf(int agent) {
    return cachedCollisionCosts[agent];
  }

  @Override
  public int footprintConflictsOf(int agent) {
    return 0;
  }

  @Override
  public GuideCounters counters() {
    return counters;
  }

  @Override
  public void setGuidePaths(int[][] paths) {
    if (paths == null || paths.length != agents) {
      return;
    }
    for (int agent = 0; agent < agents; agent++) {
      Arrays.fill(guidePaths[agent], NO_VERTEX);
      if (paths[agent] == null) {
        guidedNext[agent] = NO_VERTEX;
        continue;
      }
      int copied = Math.min(paths[agent].length, window);
      System.arraycopy(paths[agent], 0, guidePaths[agent], 0, copied);
      guidedNext[agent] = copied > 1 ? guidePaths[agent][1] : NO_VERTEX;
    }
  }

  @Override
  public int[][] currentGuidePaths() {
    return PathUtils.copyOf(guidePaths, agents);
  }

  public void saveCurrentPaths() {
    guidePathsHistory.add(currentGuidePaths());
    currentStep++;
  }

  public void clearHistory() {
    guidePathsHistory.clear();
    currentStep = 0;
  }

  public int historySize() {
    return guidePathsHistory.size();
  }

  public int currentStep() {
    return currentStep;
  }

  private boolean[] goalFlags(int[] configuration) {
    if (!clearGoalFirst) {
      return null;
    }
    boolean[] atGoal = new boolean[agents];
    for (int agent = 0; agent < agents; agent++) {
      atGoal[agent] = configuration[agent] == goals[agent];
    }
    return atGoal;
  }

  private int carryGuidePathsForwardOneStep(int[] configuration) {
    int slid = 0;
    for (int agent = 0; agent < agents; agent++) {
      if (configuration[agent] != guidePaths[agent][1]) {
        continue;
      }
      for (int time = 0; time + 1 < window; time++) {
        guidePaths[agent][time] = guidePaths[agent][time + 1];
      }
      collisions.enrollPath(agent, guidePaths[agent]);
      slid++;
    }
    return slid;
  }

  private void releaseGoalAgents(boolean[] atGoal) {
    if (atGoal == null) {
      return;
    }
    for (int agent = 0; agent < agents; agent++) {
      if (atGoal[agent]) {
        collisions.clearPath(agent, guidePaths[agent]);
      }
    }
  }

  private void replanGoalAgents(boolean[] atGoal, int[] configuration) {
    if (atGoal == null) {
      return;
    }
    for (int agent = 0; agent < agents; agent++) {
      if (!atGoal[agent]) {
        continue;
      }
      guidedNext[agent] = NO_VERTEX;
      updatePath(agent, configuration);
      collisions.enrollPath(agent, guidePaths[agent]);
    }
  }

  private void updatePath(int agent, int[] configuration) {
    int start = configuration[agent];
    int[] path = guidePaths[agent];
    if (start == goals[agent]) {
      Arrays.fill(path, start);
      cachedCollisionCosts[agent] = 0.0f;
      return;
    }

    Arrays.fill(path, NO_VERTEX);
    PriorityQueue<Node> open = new PriorityQueue<>(CHEAPEST_THEN_DEEPEST);
    closedSlots.clear();
    open.add(nodeFor(agent, start, null));
    while (!open.isEmpty()) {
      Node node = open.poll();
      if (closed[node.when][node.where] != null) {
        continue;
      }
      closed[node.when][node.where] = node;
      closedSlots.add(new int[] {node.when, node.where});
      if (node.where == goals[agent] || node.when == window - 1) {
        cachedCollisionCosts[agent] = writeBack(agent, node);
        break;
      }
      int[] candidates = roadmap.candidates(node.where);
      if (!deterministic) {
        shuffle(candidates);
      }
      int nextTime = node.when + 1;
      for (int candidate : candidates) {
        if (closed[nextTime][candidate] == null) {
          open.add(nodeFor(agent, candidate, node));
        }
      }
    }
    if (path[0] == NO_VERTEX) {
      Arrays.fill(path, start);
      cachedCollisionCosts[agent] = 0.0f;
    }
    for (int[] slot : closedSlots) {
      closed[slot[0]][slot[1]] = null;
    }
    closedSlots.clear();
  }

  private float writeBack(int agent, Node tail) {
    float total = 0.0f;
    for (Node node = tail; node != null; node = node.parent) {
      guidePaths[agent][node.when] = node.where;
      if (node.parent != null) {
        int collision = collisions.collisionCost(node.parent.where, node.where, node.parent.when);
        if (collision >= 1) {
          total += scaledCollisionCost + collision * scaledCollisionCostOrder;
        }
      }
    }
    if (tail.where == goals[agent]) {
      Arrays.fill(guidePaths[agent], tail.when + 1, window, tail.where);
    }
    return total;
  }

  private Node nodeFor(int agent, int where, Node parent) {
    int collision = parent == null ? 0
        : collisions.collisionCost(parent.where, where, parent.when);
    float g = parent == null ? 0.0f : parent.g + 1.0f;
    if (collision >= 1) {
      g += scaledCollisionCost + collision * scaledCollisionCostOrder;
    }
    return new Node(
        parent == null ? 0 : parent.when + 1,
        where,
        g,
        distTable.get(agent, where),
        parent);
  }

  private void shuffle(int[] values) {
    for (int index = values.length - 1; index > 0; index--) {
      int pick = (int) (rng.next() * (index + 1));
      int swapped = values[index];
      values[index] = values[pick];
      values[pick] = swapped;
    }
  }

  private static final class Node {

    private final int when;
    private final int where;
    private final float g;
    private final float f;
    private final Node parent;

    Node(int when, int where, float g, float h, Node parent) {
      this.when = when;
      this.where = where;
      this.g = g;
      this.f = g + h;
      this.parent = parent;
    }
  }
}
