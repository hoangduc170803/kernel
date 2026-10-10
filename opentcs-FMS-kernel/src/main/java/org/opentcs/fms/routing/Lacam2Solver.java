package org.opentcs.fms.routing;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

import org.opentcs.fms.routing.diag.SolveReport;
import org.opentcs.fms.routing.pibt.Footprint;
import org.opentcs.fms.routing.pibt.Mulberry32;
import org.opentcs.fms.routing.pibt.PibtEngine;

public final class Lacam2Solver {

  private static final org.slf4j.Logger logger = org.slf4j.LoggerFactory.getLogger(Lacam2Solver.class);

  public enum Objective {
    NONE,
    MAKESPAN,
    SUM_OF_LOSS
  }

  public static final double DEFAULT_RESTART_RATE = 0.001;

  public static final int LOOP_LIMIT_UNBOUNDED = Integer.MAX_VALUE;

  private static final int PIBT_SEED_MIX = 0x9E3779B9;

  private static final int GUIDE_SEED_MIX = 0xC2B2AE35;

  private static final float PRIORITY_TIE_SCALE = 10000.0f;

  private static final boolean SEARCH_PAST_WINDOW = Boolean
      .parseBoolean(System.getProperty("fms.mapf.searchPastWindow", "true"));

  private static final boolean TURN_AWARE_TABLE = Boolean
      .parseBoolean(System.getProperty("fms.mapf.turnAwareTable", "true"));

  private static final boolean CONSTRAINT_WEIGHT_ORDER = Boolean
      .parseBoolean(System.getProperty("fms.mapf.constraintWeightOrder", "true"));

  private static final boolean LOG_SOLUTION_STEPS = Boolean
      .parseBoolean(System.getProperty("fms.mapf.logSolutionSteps", "false"));

  public static Map<String, String> effectiveSettings() {
    return Map.of(
        "fms.mapf.searchPastWindow", String.valueOf(SEARCH_PAST_WINDOW),
        "fms.mapf.turnAwareTable", String.valueOf(TURN_AWARE_TABLE),
        "fms.mapf.constraintWeightOrder", String.valueOf(CONSTRAINT_WEIGHT_ORDER),
        "fms.mapf.logSolutionSteps", String.valueOf(LOG_SOLUTION_STEPS));
  }

  private static final int MAX_BACKSTEP_AGENTS_SHOWN = 4;

  public record Result(
      List<int[]> solution,
      boolean optimal,
      boolean reachedGoals,
      int loopCount,
      int constrainedCalls,
      int nodesGenerated,
      long runtimeMs,
      int pibtStepFailures,
      int shuttleDetections,
      AgentPriority[] executedPriorities,
      int[][] guidePaths) {

    public boolean solved() {
      return !solution.isEmpty();
    }
  }

  private final Roadmap roadmap;

  public Lacam2Solver(Roadmap roadmap) {
    this.roadmap = roadmap;
  }

  public Result solve(
      MapfInstance instance, long deadlineMs, int seed, Objective objective, double restartRate,
      int loopLimit, int window, AgentPriority[] rootPriorities, int[] startAxis,
      boolean[] loaded, int[][] seedGuidePaths, int[][] committedPrefixes,
      StaticBodies statics) {
    return solve(
        instance, deadlineMs, seed, objective, restartRate, loopLimit, window, rootPriorities,
        startAxis, loaded, seedGuidePaths, committedPrefixes, statics, null);
  }

  public Result solve(
      MapfInstance instance, long deadlineMs, int seed, Objective objective, double restartRate,
      int loopLimit, int window, AgentPriority[] rootPriorities, int[] startAxis,
      boolean[] loaded, int[][] seedGuidePaths, int[][] committedPrefixes,
      StaticBodies statics, int[] startMotion) {
    return solve(
        instance, deadlineMs, seed, objective, restartRate, loopLimit, window, rootPriorities,
        startAxis, loaded, seedGuidePaths, committedPrefixes, statics, startMotion,
        new LegalMoves(roadmap));
  }

  public Result solve(
      MapfInstance instance, long deadlineMs, int seed, Objective objective, double restartRate,
      int loopLimit, int window, AgentPriority[] rootPriorities, int[] startAxis,
      boolean[] loaded, int[][] seedGuidePaths, int[][] committedPrefixes,
      StaticBodies statics, int[] startMotion, LegalMoves moves) {
    return new Search(
        roadmap, instance, deadlineMs, seed, objective, restartRate, loopLimit, window,
        rootPriorities, startAxis, loaded, seedGuidePaths, committedPrefixes, statics,
        startMotion, Objects.requireNonNull(moves))
        .run();
  }

  static void sortByVertexWeight(int[] candidates, Roadmap roadmap) {
    for (int i = 1; i < candidates.length; i++) {
      int vertex = candidates[i];
      int weight = roadmap.weightOf(vertex);
      int j = i - 1;
      while (j >= 0 && roadmap.weightOf(candidates[j]) > weight) {
        candidates[j + 1] = candidates[j];
        j--;
      }
      candidates[j + 1] = vertex;
    }
  }

  private static final class LowNode {

    private final int[] who;
    private final int[] where;
    private final int depth;
    private final int cursor;

    LowNode() {
      this.who = new int[0];
      this.where = new int[0];
      this.depth = 0;
      this.cursor = 0;
    }

    LowNode(int[] who, int[] where, int cursor) {
      this.who = who;
      this.where = where;
      this.depth = who.length;
      this.cursor = cursor;
    }

    LowNode(LowNode parent, int agent, int vertex, int cursor) {
      this.who = append(parent.who, agent);
      this.where = append(parent.where, vertex);
      this.depth = parent.depth + 1;
      this.cursor = cursor;
    }

    private static int[] append(int[] values, int value) {
      int[] out = Arrays.copyOf(values, values.length + 1);
      out[values.length] = value;
      return out;
    }
  }

  private static final class HighNode {

    private final int[] config;
    private final int[] axis;
    private final int[] motion;
    private final int[] pinned;
    private final Set<HighNode> neighbor = new LinkedHashSet<>();
    private final AgentPriority[] priorities;
    private final int[] order;
    private final Deque<LowNode> searchTree = new ArrayDeque<>();
    private HighNode parent;
    private long costPrimary;
    private long costSecondary;
    private int depth;
    private int failedSteps;
    private int expansions;
    private final int parentFailuresWhenBorn;
    private boolean rewired;
    private int[][] guidePaths;
    private HighNode guideParent;
    private long guideParentVersion = -1;
    private long guideVersion;

    HighNode(
        int[] config, int[] axis, int[] motion, HighNode parent, DistTable dist,
        DistTable priorityDist, AgentPriority[] rootPriorities) {
      this.config = config;
      this.axis = axis;
      this.motion = motion;
      this.pinned = new int[config.length];
      Arrays.fill(this.pinned, CollisionTable.NO_VERTEX);
      this.parent = parent;
      this.costPrimary = parent == null ? 0L : parent.costPrimary + unreachedIn(config, dist);
      this.costSecondary = parent == null ? 0L : parent.costSecondary + movedBetween(parent.config, config);
      this.depth = parent == null ? 0 : parent.depth + 1;
      this.parentFailuresWhenBorn = parent == null ? 0 : parent.failedSteps;
      this.searchTree.addLast(new LowNode());

      if (parent != null) {
        parent.neighbor.add(this);
      }

      int n = config.length;
      this.priorities = new AgentPriority[n];
      for (int i = 0; i < n; i++) {
        boolean atGoal = dist.get(i, config[i]) == 0;
        if (parent == null) {
          AgentPriority seeded = rootPriorities == null || rootPriorities[i] == null
              ? AgentPriority.NEWCOMER
              : rootPriorities[i];
          priorities[i] = new AgentPriority(
              seeded.retreating(), seeded.inGoalLane(), seeded.aged(), seeded.settled(),
              (float) priorityDist.get(i, config[i]) / PRIORITY_TIE_SCALE,
              seeded.previousOrderRank());
        } else {
          AgentPriority inherited = parent.priorities[i];
          priorities[i] = new AgentPriority(
              inherited.retreating(),
              inherited.inGoalLane(),
              atGoal ? 0 : inherited.aged() + 1,
              atGoal ? inherited.settled() - 1 : 0,
              inherited.tie(),
              inherited.previousOrderRank());
        }
      }
      this.order = orderByPriority(priorities);
    }

    private static int[] orderByPriority(AgentPriority[] priorities) {
      Integer[] indices = new Integer[priorities.length];
      for (int i = 0; i < indices.length; i++) {
        indices[i] = i;
      }
      Arrays.sort(indices, (a, b) -> priorities[b].compareTo(priorities[a]));
      int[] order = new int[indices.length];
      for (int i = 0; i < order.length; i++) {
        order[i] = indices[i];
      }
      return order;
    }
  }

  private static long unreachedIn(int[] config, DistTable dist) {
    long unreached = 0L;
    for (int agent = 0; agent < config.length; agent++) {
      if (dist.get(agent, config[agent]) != 0) {
        unreached++;
      }
    }
    return unreached;
  }

  private static long movedBetween(int[] from, int[] to) {
    long moved = 0L;
    for (int agent = 0; agent < from.length; agent++) {
      if (from[agent] != to[agent]) {
        moved++;
      }
    }
    return moved;
  }

  private static boolean cheaperThan(HighNode candidate, HighNode incumbent) {
    return candidate.costPrimary != incumbent.costPrimary
        ? candidate.costPrimary < incumbent.costPrimary
        : candidate.costSecondary < incumbent.costSecondary;
  }

  private static final class ConfigKey {

    private final int[] config;
    private final int[] axis;
    private final int[] motion;
    private final int hash;

    ConfigKey(int[] config, int[] axis, int[] motion) {
      this.config = config;
      this.axis = axis;
      this.motion = motion;
      this.hash = 31 * (31 * Arrays.hashCode(config) + Arrays.hashCode(axis))
          + Arrays.hashCode(motion);
    }

    @Override
    public boolean equals(Object other) {
      return other instanceof ConfigKey key
          && Arrays.equals(config, key.config)
          && Arrays.equals(axis, key.axis)
          && Arrays.equals(motion, key.motion);
    }

    @Override
    public int hashCode() {
      return hash;
    }
  }

  private static final class Search {

    private final Roadmap roadmap;
    private final LegalMoves moves;
    private final int n;
    private final int[] starts;
    private final int[] goals;
    private final DistTable dist;
    private final DistTable priorityDist;
    private final Objective objective;
    private final double restartRate;
    private final Mulberry32 rng;
    private final Guidance guide;
    private final int[][] initialGuidePaths;
    private final CommittedPrefix[] committedPrefixes;
    private final StaticBodies statics;
    private final PibtEngine pibt;
    private final long deadlineAt;
    private final int loopLimit;
    private final int window;
    private final AgentPriority[] rootPriorities;
    private final Footprint footprint;
    private final int[] startAxis;
    private final int[] startMotion;
    private final boolean[] loaded;
    private int loopCount;
    private int guideConstructions;

    private int[][] rootGuidePaths;
    private int constrainedCalls;
    private int prunedByLimit;
    private int pushBlockedByLimit;
    private int rewriteRelaxations;
    private int rewriteParentChanges;
    private int pibtStepFailures;
    private int shuttleDetections;
    private int shuttleLines;
    private int constraintsGenerated;
    private int constraintsForcingStay;
    private int constraintsAwayFromGoal;
    private int pinnedMoves;
    private int pinsDropped;
    private int commitmentViolations;

    Search(
        Roadmap roadmap, MapfInstance instance, long deadlineMs, int seed, Objective objective,
        double restartRate, int loopLimit, int window, AgentPriority[] rootPriorities,
        int[] startAxis, boolean[] loaded, int[][] seedGuidePaths,
        int[][] committedPrefixes, StaticBodies statics, int[] startMotion, LegalMoves moves) {
      this.statics = statics == null ? StaticBodies.NONE : statics;
      this.committedPrefixes = CommittedPrefix.copyOf(committedPrefixes, instance.starts().length);
      this.footprint = Footprint.enabled() ? new Footprint(roadmap) : null;
      this.startAxis = startAxis;
      this.startMotion = startMotion;
      this.loaded = loaded;
      this.loopLimit = loopLimit;
      this.window = window;
      this.rootPriorities = rootPriorities;
      this.roadmap = roadmap;
      this.moves = moves;
      this.starts = instance.starts().clone();
      this.goals = instance.goals().clone();
      this.n = starts.length;
      this.objective = objective;
      this.restartRate = restartRate;
      this.rng = new Mulberry32(seed);
      this.dist = TURN_AWARE_TABLE
          ? DistTable.turnAware(roadmap, goals)
          : DistTable.weighted(roadmap, goals);
      this.priorityDist = DistTable.work(roadmap, goals);
      this.guide = createGuide(seed);
      if (guide != null && seedGuidePaths != null) {
        guide.setGuidePaths(seedGuidePaths);
      }
      this.initialGuidePaths = guide == null ? null : guide.currentGuidePaths();
      this.pibt = new PibtEngine(roadmap, n, seed ^ PIBT_SEED_MIX, dist, guide, footprint, moves);
      this.deadlineAt = deadlineMs > 0 ? System.currentTimeMillis() + deadlineMs : Long.MAX_VALUE;
    }

    private Guidance createGuide(int seed) {
      if (!GuideSettings.enabled()) {
        return null;
      }
      String profile = GuideSettings.profile();
      if (GuideSettings.LLLG_PROFILE.equals(profile)) {
        return new LllgLocalGuide(
            roadmap, DistTable.weighted(roadmap, goals), goals, n, seed ^ GUIDE_SEED_MIX);
      }
      if (GuideSettings.FMS_PROFILE.equals(profile)) {
        return FmsLocalGuide.builder(roadmap, dist, goals, seed ^ GUIDE_SEED_MIX)
            .footprint(footprint)
            .loaded(loaded)
            .committedPrefixes(committedPrefixes)
            .moves(moves)
            .statics(statics)
            .build();
      }
      throw new IllegalArgumentException("Unknown fms.guide.profile: " + profile);
    }

    private int[] rootAxis() {
      if (startAxis != null) {
        return startAxis.clone();
      }
      int[] axes = new int[n];
      Arrays.fill(axes, Roadmap.DIR_NONE);
      return axes;
    }

    private int[] axisAfter(HighNode node, int[] next) {
      int[] axes = new int[n];
      for (int i = 0; i < n; i++) {
        axes[i] = roadmap.noseAfter(node.axis[i], node.config[i], next[i]);
      }
      return axes;
    }

    private int[] rootMotion() {
      int[] motions = new int[n];
      Arrays.fill(motions, Roadmap.DIR_NONE);
      if (startMotion == null) {
        return motions;
      }
      System.arraycopy(startMotion, 0, motions, 0, Math.min(n, startMotion.length));
      return motions;
    }

    private int[] motionAfter(HighNode node, int[] next) {
      int[] motions = new int[n];
      for (int i = 0; i < n; i++) {
        motions[i] = roadmap.motionAfter(node.config[i], next[i]);
      }
      return motions;
    }

    Result run() {
      long startedAt = System.currentTimeMillis();
      Deque<HighNode> open = new ArrayDeque<>();
      Map<ConfigKey, HighNode> explored = new HashMap<>();

      HighNode root = new HighNode(
          starts, rootAxis(), rootMotion(), null, dist, priorityDist, rootPriorities);
      pinCommittedMoves(root);
      logAgentOrder(root);
      open.push(root);
      explored.put(new ConfigKey(starts, root.axis, root.motion), root);
      HighNode bestGoal = null;
      HighNode bestWindowed = null;

      while (!open.isEmpty() && !expired()) {
        loopCount++;
        HighNode node = open.peek();

        if (node.searchTree.isEmpty()) {
          open.pop();
          continue;
        }
        HighNode limit = pruningLimit(bestGoal, bestWindowed);
        if (limit != null && !cheaperThan(node, limit)) {
          prunedByLimit++;
          open.pop();
          continue;
        }
        if (Arrays.equals(node.config, goals)) {
          if (bestGoal == null || cheaperThan(node, bestGoal)) {
            bestGoal = node;
          }
          if (objective == Objective.NONE) {
            break;
          }
          open.pop();
          continue;
        }
        if (node.depth >= window) {
          if (bestWindowed == null || cheaperThan(node, bestWindowed)) {
            bestWindowed = node;
          }
          if (objective == Objective.NONE) {
            break;
          }
          open.pop();
          continue;
        }

        LowNode low = node.searchTree.pollFirst();
        if (low.depth > 0) {
          constrainedCalls++;
        }
        expandLowLevelTree(node, low);
        node.expansions++;

        if (guide != null) {
          activateGuide(node);
        }

        int[] next = pibt.step(
            node.config, node.order, constraintsOf(low), node.axis, loaded,
            statics, node.motion);
        if (next == null) {
          node.failedSteps++;
          pibtStepFailures++;
          continue;
        }
        logCommittedPrefixDeviation(node, next);

        int[] nextAxis = axisAfter(node, next);
        int[] nextMotion = motionAfter(node, next);
        ConfigKey key = new ConfigKey(next, nextAxis, nextMotion);
        HighNode known = explored.get(key);
        if (known != null) {
          rewrite(node, known, limit, open);
          HighNode reinserted = rng.next() >= restartRate ? known : root;
          if (limit == null || cheaperThan(reinserted, limit)) {
            open.push(reinserted);
          } else {
            pushBlockedByLimit++;
          }
        } else {
          HighNode child = new HighNode(
              next, nextAxis, nextMotion, node, dist, priorityDist, null);
          pinCommittedMoves(child);
          detectShuttles(child);
          explored.put(key, child);
          if (limit == null || cheaperThan(child, limit)) {
            open.push(child);
          } else {
            pushBlockedByLimit++;
          }
        }
      }

      HighNode terminal = bestGoal != null ? bestGoal : bestWindowed;
      List<int[]> solution = new ArrayList<>();
      List<HighNode> chain = new ArrayList<>();
      if (terminal != null) {
        for (HighNode node = terminal; node != null; node = node.parent) {
          solution.add(node.config);
          chain.add(node);
        }
        Collections.reverse(solution);
        Collections.reverse(chain);
      }
      logCommitmentViolations(solution);
      boolean reachedGoals = bestGoal != null;
      logBackstepAndGuidance(solution);
      logRetryOrigin(chain);
      logCostComparisons();
      logFootprintActivity();
      logRootBacksteps(chain);
      logSolution(chain);

      return new Result(
          List.copyOf(solution),
          reachedGoals && open.isEmpty(),
          reachedGoals,
          loopCount,
          constrainedCalls,
          explored.size(),
          System.currentTimeMillis() - startedAt,
          pibtStepFailures,
          shuttleDetections,
          chain.isEmpty() ? null : chain.get(Math.min(1, chain.size() - 1)).priorities,
          rootGuidePaths);
    }

    private void detectShuttles(HighNode child) {
      for (int agent = 0; agent < n; agent++) {
        if (child.config[agent] == child.parent.config[agent]
            || child.config[agent] == goals[agent]) {
          continue;
        }
        boolean repeatedState = false;
        for (HighNode ancestor = child.parent;
            ancestor != null && child.depth - ancestor.depth <= 3;
            ancestor = ancestor.parent) {
          if (child.config[agent] == ancestor.config[agent]
              && child.axis[agent] == ancestor.axis[agent]) {
            repeatedState = true;
            break;
          }
        }
        if (!repeatedState) {
          continue;
        }
        List<ShuttleCycleDetector.State> ancestors = new ArrayList<>(3);
        for (HighNode ancestor = child.parent;
            ancestor != null && child.depth - ancestor.depth <= 3;
            ancestor = ancestor.parent) {
          ancestors.add(new ShuttleCycleDetector.State(
              ancestor.depth, ancestor.config, ancestor.axis));
        }
        ShuttleCycleDetector.Match match = ShuttleCycleDetector.detect(
            roadmap, footprint, goals, agent,
            new ShuttleCycleDetector.State(child.depth, child.config, child.axis),
            new ShuttleCycleDetector.State(
                child.parent.depth, child.parent.config, child.parent.axis),
            ancestors, statics, 3);
        if (match == null) {
          continue;
        }
        shuttleDetections++;
        if (shuttleLines >= 20) {
          continue;
        }
        shuttleLines++;
        logger.info(
            "[MAPF shuttle] a{} returned to {} axis={} at depth {} (ancestor depth {}),"
                + " neighbourhood unchanged {}, jointKeyNew=true",
            agent, roadmap.nameOf(child.config[agent]), axisName(child.axis[agent]),
            child.depth, match.ancestorDepth(), match.neighborhood());
      }
    }

    private static String axisName(int axis) {
      return switch (axis) {
        case Roadmap.AXIS_X -> "X";
        case Roadmap.AXIS_Y -> "Y";
        default -> "none";
      };
    }

    private void logBackstepAndGuidance(List<int[]> solution) {
      int transitions = solution.size() - 1;
      if (transitions < 1) {
        return;
      }
      int rootBack = backwardAgents(solution.get(0), solution.get(1));
      int laterBack = 0;
      for (int t = 1; t < transitions; t++) {
        laterBack += backwardAgents(solution.get(t), solution.get(t + 1));
      }
      int laterSlots = (transitions - 1) * n;
      logger.debug(
          "[MAPF backstep] agents={} transitions={} rootBack={}/{} ({}%) laterBack={}/{} ({}%)",
          n, transitions,
          rootBack, n, ratio(rootBack, n),
          laterBack, laterSlots, ratio(laterBack, laterSlots));
    }

    private void logCommittedPrefixDeviation(HighNode node, int[] next) {
      for (int agent = 0; agent < n; agent++) {
        CommittedPrefix prefix = committedPrefixes[agent];
        if (!prefix.isActiveAt(node.depth, node.config[agent])) {
          continue;
        }
        int expected = prefix.committedMoveAt(node.depth);
        if (next[agent] == expected) {
          continue;
        }
        int guided = guide == null ? CollisionTable.NO_VERTEX
            : guide.guidedConfiguration()[agent];
        logger.warn(
            "[MAPF committed-prefix-deviation] a{} depth={} current={} expected={} actual={}"
                + " guideWanted={}",
            agent, node.depth, roadmap.nameOf(node.config[agent]), roadmap.nameOf(expected),
            roadmap.nameOf(next[agent]),
            guided == CollisionTable.NO_VERTEX ? "none" : roadmap.nameOf(guided));
      }
    }

    private void pinCommittedMoves(HighNode node) {
      Map<Integer, Integer> pinnedByTarget = new HashMap<>();
      for (int agent : node.order) {
        CommittedPrefix prefix = committedPrefixes[agent];
        if (!prefix.isActiveAt(node.depth, node.config[agent])) {
          continue;
        }
        int target = prefix.committedMoveAt(node.depth);
        if (!isAdjacentOrSame(node.config[agent], target)) {
          dropPin(agent, node, target, "not adjacent on the planning roadmap");
          continue;
        }
        int occupant = standingAgentAt(node.config, target, agent);
        if (occupant != CollisionTable.NO_VERTEX) {
          dropPin(agent, node, target, "occupied by a" + occupant);
          continue;
        }
        Integer earlierAgent = pinnedByTarget.get(target);
        if (earlierAgent != null) {
          dropPin(agent, node, target, "same target as a" + earlierAgent);
          continue;
        }
        node.pinned[agent] = target;
        pinnedByTarget.put(target, agent);
        pinnedMoves++;
      }
      int pinCount = 0;
      for (int agent : node.order) {
        if (node.pinned[agent] != CollisionTable.NO_VERTEX) {
          pinCount++;
        }
      }
      if (pinCount == 0) {
        return;
      }
      int[] who = new int[pinCount];
      int[] where = new int[pinCount];
      int index = 0;
      for (int agent : node.order) {
        if (node.pinned[agent] == CollisionTable.NO_VERTEX) {
          continue;
        }
        who[index] = agent;
        where[index] = node.pinned[agent];
        index++;
      }
      node.searchTree.clear();
      node.searchTree.addLast(new LowNode(who, where, 0));
    }

    private boolean isAdjacentOrSame(int from, int to) {
      return from == to || contains(roadmap.neighborsOf(from), to);
    }

    private void dropPin(int agent, HighNode node, int target, String reason) {
      pinsDropped++;
      logger.warn(
          "[MAPF prefix-pin] a{} depth={} at={} committed={} not pinned: {}",
          agent, node.depth, roadmap.nameOf(node.config[agent]), roadmap.nameOf(target), reason);
    }

    private static int standingAgentAt(int[] config, int target, int movingAgent) {
      for (int agent = 0; agent < config.length; agent++) {
        if (agent != movingAgent && config[agent] == target) {
          return agent;
        }
      }
      return CollisionTable.NO_VERTEX;
    }

    private static boolean contains(int[] values, int target) {
      for (int value : values) {
        if (value == target) {
          return true;
        }
      }
      return false;
    }

    private void logCommitmentViolations(List<int[]> solution) {
      boolean[] logged = new boolean[n];
      for (int agent = 0; agent < n; agent++) {
        CommittedPrefix prefix = committedPrefixes[agent];
        for (int depth = 0; depth + 1 < prefix.length() && depth + 1 < solution.size(); depth++) {
          if (!prefix.isViolatedAt(
              depth, solution.get(depth)[agent], solution.get(depth + 1)[agent])) {
            continue;
          }
          commitmentViolations++;
          if (logged[agent]) {
            continue;
          }
          logged[agent] = true;
          logger.warn(
              "[MAPF prefix-pin] solution violates commitment a{} depth={} committed={} planned={}",
              agent, depth, roadmap.nameOf(prefix.committedMoveAt(depth)),
              roadmap.nameOf(solution.get(depth + 1)[agent]));
        }
      }
    }

    private void activateGuide(HighNode node) {
      if (guideIsCurrent(node)) {
        guide.setGuidePaths(node.guidePaths);
        return;
      }
      if (node.parent != null && !guideIsCurrent(node.parent)) {
        activateGuide(node.parent);
      }
      long parentVersion = node.parent == null ? 0 : node.parent.guideVersion;
      guide.setGuidePaths(node.parent == null ? initialGuidePaths : node.parent.guidePaths);
      guide.construct(node.config, node.axis, node.motion, node.order, node.depth);
      guideConstructions++;
      node.guidePaths = guide.currentGuidePaths();
      node.guideParent = node.parent;
      node.guideParentVersion = parentVersion;
      node.guideVersion++;
      if (node.parent == null && rootGuidePaths == null) {
        rootGuidePaths = PathUtils.copyOf(node.guidePaths, n);
      }
    }

    private boolean guideIsCurrent(HighNode node) {
      if (node.guidePaths == null || node.guideParent != node.parent) {
        return false;
      }
      return node.parent == null
          ? node.guideParentVersion == 0
          : node.guideParentVersion == node.parent.guideVersion && guideIsCurrent(node.parent);
    }

    private int backwardAgents(int[] before, int[] after) {
      int count = 0;
      for (int i = 0; i < n; i++) {
        if (dist.get(i, after[i]) > dist.get(i, before[i])) {
          count++;
        }
      }
      return count;
    }

    private void logRetryOrigin(List<HighNode> chain) {
      if (chain.size() < 2) {
        return;
      }
      int afterRetry = 0;
      int afterRetryBackward = 0;
      int firstTry = 0;
      int firstTryBackward = 0;
      int rewired = 0;
      for (int t = 1; t < chain.size(); t++) {
        HighNode child = chain.get(t);
        boolean backward = backwardAgents(chain.get(t - 1).config, child.config) > 0;
        rewired += child.rewired ? 1 : 0;
        if (child.parentFailuresWhenBorn > 0) {
          afterRetry++;
          afterRetryBackward += backward ? 1 : 0;
        } else {
          firstTry++;
          firstTryBackward += backward ? 1 : 0;
        }
      }
      logger.debug(
          "[MAPF retry-origin] agents={} transitions={} afterNullPibt={} withBackstep={} ({}%)"
              + " firstTry={} withBackstep={} ({}%) rewiredParents={} totalPibtFailures={}",
          n, chain.size() - 1,
          afterRetry, afterRetryBackward, ratio(afterRetryBackward, afterRetry),
          firstTry, firstTryBackward, ratio(firstTryBackward, firstTry),
          rewired, pibtStepFailures);
    }

    private void logFootprintActivity() {
      if (footprint == null) {
        return;
      }
      int carrying = 0;
      if (loaded != null) {
        for (boolean isLoaded : loaded) {
          if (isLoaded) {
            carrying++;
          }
        }
      }
      logger.debug(
          "[MAPF footprint] agents={} loaded={} turnsConsidered={} candidateRejections={}"
              + " pushes={} configurationsRejected={} pibtStepFailures={} loops={}",
          n, carrying, pibt.counters().turnsConsidered, pibt.counters().footprintRejections,
          pibt.counters().footprintPushes, pibt.counters().overlappingConfigurations, pibtStepFailures, loopCount);
      if (pibt.counters().turnsConsidered == 0 && n > 1) {
        logger.warn(
            "[MAPF footprint] no candidate move was classified as a turn this solve; every"
                + " agent axis is probably AXIS_NONE, so the swept-disc half of the model is"
                + " inert. Check the axis supplied by FMSDispatcher.");
      }
    }

    private void logAgentOrder(HighNode root) {
      if (!logger.isDebugEnabled()) {
        return;
      }
      StringBuilder out = new StringBuilder();
      for (int k = 0; k < root.order.length; k++) {
        int agent = root.order[k];
        AgentPriority p = root.priorities[agent];
        out.append(k == 0 ? "" : "  ")
            .append('#').append(k).append("=a").append(agent)
            .append('@').append(roadmap.nameOf(root.config[agent]))
            .append("[retreat=").append(p.retreating())
            .append(" goalLane=").append(p.inGoalLane())
            .append(" aged=").append(p.aged())
            .append(" settled=").append(p.settled())
            .append(" tie=").append(p.tie())
            .append(" (turnAwareDist=").append(dist.get(agent, root.config[agent])).append(')')
            .append(']');
      }
      logger.debug("[LACAM order] served first to last: {}", out);
    }

    private void logCostComparisons() {
      if (logger.isDebugEnabled()) {
        logger.debug(
            "[MAPF cost-compare] loops={} prunedByLimit={} pushBlockedByLimit={}"
                + " rewriteRelaxations={} rewriteParentChanges={}",
            loopCount, prunedByLimit, pushBlockedByLimit,
            rewriteRelaxations, rewriteParentChanges);
        logger.debug(
            "[LACAM constraint] generated={} forcingStay={} awayFromGoal={} constrainedPibtCalls={}"
                + " pinned={} pinDropped={} commitmentViolations={}",
            constraintsGenerated, constraintsForcingStay, constraintsAwayFromGoal,
            constrainedCalls, pinnedMoves, pinsDropped, commitmentViolations);
        logger.debug(
            "[PIBT choice] {}", SolveReport.pibtChoices(pibt.counters(), PibtEngine.hindranceEnabled()));
        logGuideActivity();
      }
      logPibtFailures();
    }

    private void logPibtFailures() {
      if (pibtStepFailures == 0) {
        return;
      }
      logger.warn(
          "[PIBT no-config] {} of {} PIBT calls produced no configuration;{}",
          pibtStepFailures, loopCount, SolveReport.pibtFailures(pibt.counters()));
    }

    private void logGuideActivity() {
      if (guide == null || !logger.isDebugEnabled()) {
        return;
      }
      logger.debug(
          "[MAPF guide] {}",
          SolveReport.guideActivity(
              guide.counters(), n, guide.window(), guideConstructions,
              pibt.counters().guidanceDecisions, pibt.counters().hindranceDecisions));
      int[][] paths = rootGuidePaths;
      for (int agent = 0; agent < n; agent++) {
        logger.debug(
            "[MAPF guide-path] a{} start={} goal={} nextStep={} guide={}",
            agent, roadmap.nameOf(starts[agent]), roadmap.nameOf(goals[agent]),
            nextStepOf(paths, agent), runLengthNames(paths, agent));
      }
    }

    private String nextStepOf(int[][] paths, int agent) {
      if (paths == null || paths[agent] == null || paths[agent].length < 2
          || paths[agent][1] < 0) {
        return "none";
      }
      return paths[agent][1] == starts[agent]
          ? "STAY@" + roadmap.nameOf(paths[agent][1])
          : roadmap.nameOf(paths[agent][1]);
    }

    private String runLengthNames(int[][] paths, int agent) {
      if (paths == null || paths[agent] == null || paths[agent].length == 0) {
        return "[]";
      }
      int[] path = paths[agent];
      StringBuilder out = new StringBuilder("[");
      int index = 0;
      while (index < path.length) {
        int run = 1;
        while (index + run < path.length && path[index + run] == path[index]) {
          run++;
        }
        if (index > 0) {
          out.append(", ");
        }
        out.append(path[index] < 0 ? "?" : roadmap.nameOf(path[index]));
        if (run > 1) {
          out.append('x').append(run);
        }
        index += run;
      }
      return out.append(']').toString();
    }

    private String stayAlternative(HighNode parent, HighNode chosen, int agent) {
      int stayed = 0;
      int advanced = 0;
      long bestStayF = Long.MAX_VALUE;
      for (HighNode option : parent.neighbor) {
        int at = option.config[agent];
        if (at == parent.config[agent]) {
          stayed++;
          bestStayF = Math.min(bestStayF, option.costSecondary);
        } else if (dist.get(agent, at) < dist.get(agent, parent.config[agent])) {
          advanced++;
        }
      }
      return String.format(" [v%d parentExpansions=%d altsGenerated=%d stayAlts=%d fwdAlts=%d"
          + " bestStayMoves=%s chosenMoves=%d]",
          agent, parent.expansions, parent.neighbor.size(), stayed, advanced,
          bestStayF == Long.MAX_VALUE ? "none" : String.valueOf(bestStayF), chosen.costSecondary);
    }

    private String backstepReason(int[] before, int[] after, int agent) {
      StringBuilder why = new StringBuilder();
      int vacated = before[agent];
      for (int j = 0; j < n; j++) {
        if (j != agent && after[j] == vacated) {
          why.append(" evictedBy=v").append(j);
        }
      }
      for (int forward : roadmap.neighborsOf(vacated)) {
        if (dist.get(agent, forward) >= dist.get(agent, vacated)) {
          continue;
        }
        for (int j = 0; j < n; j++) {
          if (j == agent) {
            continue;
          }
          if (before[j] == forward && after[j] == vacated) {
            why.append(" fwd:").append(roadmap.nameOf(forward)).append("=headOn.v").append(j);
          } else if (after[j] == forward) {
            why.append(" fwd:").append(roadmap.nameOf(forward)).append("=takenBy.v").append(j);
          }
        }
      }
      return why.toString();
    }

    private void logRootBacksteps(List<HighNode> chain) {
      if (chain.size() < 2) {
        return;
      }
      HighNode parent = chain.get(0);
      HighNode child = chain.get(1);
      StringBuilder out = new StringBuilder();
      int total = 0;
      int shown = 0;
      for (int i = 0; i < n; i++) {
        if (dist.get(i, child.config[i]) <= dist.get(i, parent.config[i])) {
          continue;
        }
        total++;
        if (shown < MAX_BACKSTEP_AGENTS_SHOWN) {
          shown++;
          out.append(String.format(" v%d:%s->%s%s%s", i,
              roadmap.nameOf(parent.config[i]), roadmap.nameOf(child.config[i]),
              backstepReason(parent.config, child.config, i),
              stayAlternative(parent, child, i)));
        }
      }
      if (total == 0) {
        return;
      }
      logger.info(
          "[MAPF backstep-root] {} of {} agent(s) step AWAY from their goal on the one transition"
              + " that actually gets executed. A reason names the agent that pushed them; an EMPTY"
              + " reason means no other agent explains it, so the retreat was ranked first by the"
              + " guide. stayAlts=0 with altsGenerated=1 means no alternative was ever expanded,"
              + " not that standing still was impossible. |{}",
          total, n, out);
    }

    private void logSolution(List<HighNode> chain) {
      if (!LOG_SOLUTION_STEPS || chain.isEmpty()) {
        return;
      }
      HighNode terminal = chain.get(chain.size() - 1);
      logger.info(
          "[MAPF solution] agents={} length={} unreached={} moves={} objective={}",
          n, chain.size() - 1, terminal.costPrimary, terminal.costSecondary, objective);
      for (int t = 1; t < chain.size(); t++) {
        HighNode child = chain.get(t);
        int[] before = chain.get(t - 1).config;
        int[] after = child.config;
        StringBuilder moves = new StringBuilder();
        int moved = 0;
        for (int i = 0; i < n; i++) {
          if (before[i] == after[i]) {
            continue;
          }
          moved++;
          moves.append(String.format(" v%d:%s->%s%s", i,
              roadmap.nameOf(before[i]), roadmap.nameOf(after[i]),
              dist.get(i, after[i]) > dist.get(i, before[i])
                  ? "(BACK" + backstepReason(before, after, i) + ")"
                      + stayAlternative(chain.get(t - 1), child, i)
                  : ""));
        }
        logger.info(
            "[MAPF step] t={} unreached={} moves={} moved={} held={} backward={} parentRetries={}"
                + " rewired={} |{}",
            t, child.costPrimary, child.costSecondary, moved, n - moved,
            backwardAgents(before, after),
            child.parentFailuresWhenBorn, child.rewired, moves);
      }
    }

    private static String ratio(int part, int whole) {
      return whole <= 0 ? "n/a" : String.format("%.1f", 100.0 * part / whole);
    }

    private static HighNode pruningLimit(HighNode bestGoal, HighNode bestWindowed) {
      if (bestGoal != null) {
        return bestGoal;
      }
      return SEARCH_PAST_WINDOW ? null : bestWindowed;
    }

    private boolean expired() {
      return loopCount >= loopLimit || System.currentTimeMillis() >= deadlineAt;
    }

    private void expandLowLevelTree(HighNode node, LowNode low) {
      int cursor = low.cursor;
      while (cursor < n && node.pinned[node.order[cursor]] != CollisionTable.NO_VERTEX) {
        cursor++;
      }
      if (cursor >= n) {
        return;
      }
      int agent = node.order[cursor];
      int[] candidates = moves.forAgent(agent, node.config[agent], node.motion[agent]);
      shuffle(candidates);
      if (CONSTRAINT_WEIGHT_ORDER) {
        sortByVertexWeight(candidates, roadmap);
      }
      for (int vertex : candidates) {
        node.searchTree.addLast(new LowNode(low, agent, vertex, cursor + 1));
        constraintsGenerated++;
        if (vertex == node.config[agent]) {
          constraintsForcingStay++;
        } else if (dist.get(agent, vertex) > dist.get(agent, node.config[agent])) {
          constraintsAwayFromGoal++;
        }
      }
    }

    private void shuffle(int[] values) {
      for (int i = values.length - 1; i > 0; i--) {
        int j = (int) (rng.next() * (i + 1));
        int swapped = values[i];
        values[i] = values[j];
        values[j] = swapped;
      }
    }

    private List<PibtEngine.Constraint> constraintsOf(LowNode low) {
      List<PibtEngine.Constraint> constraints = new ArrayList<>(low.depth);
      for (int k = 0; k < low.depth; k++) {
        constraints.add(new PibtEngine.Constraint(low.who[k], low.where[k]));
      }
      return constraints;
    }

    private void rewrite(HighNode from, HighNode to, HighNode limit, Deque<HighNode> open) {
      from.neighbor.add(to);

      Deque<HighNode> queue = new ArrayDeque<>();
      queue.addLast(from);
      while (!queue.isEmpty()) {
        HighNode current = queue.pollFirst();
        for (HighNode successor : current.neighbor) {
          long primary = current.costPrimary + unreachedIn(successor.config, dist);
          long secondary = current.costSecondary + movedBetween(current.config, successor.config);
          if (primary != successor.costPrimary
              ? primary < successor.costPrimary
              : secondary < successor.costSecondary) {
            rewriteRelaxations++;
            rewriteParentChanges += successor.parent != current ? 1 : 0;
            successor.costPrimary = primary;
            successor.costSecondary = secondary;
            successor.rewired |= successor.parent != current;
            successor.parent = current;
            successor.depth = current.depth + 1;
            queue.addLast(successor);
            if (limit == null || cheaperThan(successor, limit)) {
              open.push(successor);
            }
          }
        }
      }
    }

  }
}
