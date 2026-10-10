package org.opentcs.fms.routing;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

import org.opentcs.fms.routing.diag.GuideCounters;
import org.opentcs.fms.routing.pibt.Footprint;
import org.opentcs.fms.routing.pibt.Mulberry32;

public final class FmsLocalGuide implements Guidance {

  private static final int NO_VERTEX = CollisionTable.NO_VERTEX;

  private static final int STATE_SLOTS = 14;

  private static final int INCONSISTENT_START_SLOT = STATE_SLOTS - 1;

  private static final int[] SLOT_BY_NOSE_AND_MOTION = stateSlots();

  private static int[] stateSlots() {
    int[] slots = new int[(Roadmap.NOSES + 1) * (Roadmap.NOSES + 1)];
    Arrays.fill(slots, INCONSISTENT_START_SLOT);
    int next = 0;
    for (int nose = Roadmap.DIR_NONE; nose < Roadmap.NOSES; nose++) {
      slots[stateIndex(nose, Roadmap.DIR_NONE)] = next++;
    }
    for (int motion = 0; motion < Roadmap.NOSES; motion++) {
      slots[stateIndex(motion, motion)] = next++;
      slots[stateIndex(Roadmap.opposite(motion), motion)] = next++;
    }
    return slots;
  }

  private static int stateIndex(int nose, int motion) {
    return (nose + 1) * (Roadmap.NOSES + 1) + motion + 1;
  }

  private static final int GUIDE_SEED_MIX = 0x85EBCA6B;

  private static final Comparator<GuideNode> CHEAPEST_THEN_STILLEST_THEN_DEEPEST = (a, b) -> {
    if (a.f != b.f) {
      return a.f < b.f ? -1 : 1;
    }
    if (a.moves != b.moves) {
      return a.moves < b.moves ? -1 : 1;
    }
    if (a.g != b.g) {
      return a.g > b.g ? -1 : 1;
    }
    return 0;
  };

  private final boolean on = GuideSettings.enabled();

  private final int configuredWindow = Integer.getInteger("fms.guide.window", 20);

  private final int numRefine = Integer.getInteger("fms.guide.numRefine", 2);

  private final float collisionCost = Float.parseFloat(System.getProperty("fms.guide.collisionCost", "3.0"));

  private final float collisionCostOrder = Float.parseFloat(System.getProperty("fms.guide.collisionCostOrder", "1e-7"));

  private final float footprintCost = Float.parseFloat(System.getProperty("fms.guide.footprintCost", "2.0"));

  private final int restingBodyTicks = Integer.getInteger("fms.guide.restingBodyTicks", 2);

  private final boolean deterministic = Boolean.parseBoolean(System.getProperty("fms.guide.deterministic", "false"));

  private final boolean clearGoalFirst = Boolean.parseBoolean(System.getProperty("fms.guide.clearGoalFirst", "true"));

  private final boolean reserveFootprint = Boolean
      .parseBoolean(System.getProperty("fms.guide.reserveFootprint", "true"));

  /**
   * Emits only rollout steps where rebuilding a carried guide changes its
   * immediate advice.
   * Disabled by default because a joint solve constructs guides at every search
   * depth.
   */
  private final boolean traceRolloutCarry = Boolean
      .parseBoolean(System.getProperty("fms.guide.traceRolloutCarry", "true"));

  private final Roadmap roadmap;
  private final DistTable distTable;
  private final int[] goals;
  private final int agents;
  private final int window;
  private final Mulberry32 rng;
  private final CollisionTable collisions;
  private final GuideFootprintReservations footprintReservations;
  private final boolean[] loaded;
  private final CommittedPrefix[] committedPrefixes;
  private final int[][] guidePaths;
  private final GuideNode[][] closed;
  private final List<int[]> closedSlots = new ArrayList<>();
  private final int[] guidedNext;
  private final float scaledCollisionCost;
  private final float scaledCollisionCostOrder;
  private final float scaledFootprintCost;
  private final float[] cachedCollisionCosts;
  private final int[] cachedFootprintConflicts;
  private final List<int[][]> guidePathsHistory = new ArrayList<>();

  private int[] standingAt;

  private int currentStep;

  private final GuideCounters counters = new GuideCounters();
  private final LocalGuideLogger guideLogger;

  private final LegalMoves moves;

  private final Footprint footprint;

  private final StaticBodies statics;

  public static Builder builder(Roadmap roadmap, DistTable distTable, int[] goals, int seed) {
    return new Builder(roadmap, distTable, goals, seed);
  }

  private FmsLocalGuide(Builder builder) {
    int agents = builder.goals.length;
    this.moves = builder.moves;
    this.footprint = builder.footprint;
    this.statics = builder.statics == null ? StaticBodies.NONE : builder.statics;
    this.roadmap = builder.roadmap;
    this.distTable = builder.distTable;
    this.goals = builder.goals.clone();
    this.agents = agents;
    this.window = Math.max(2, configuredWindow);
    this.rng = new Mulberry32(builder.seed ^ GUIDE_SEED_MIX);
    this.collisions = new CollisionTable(builder.roadmap.size(), true, false);
    this.footprintReservations = reserveFootprint && builder.footprint != null
        ? new GuideFootprintReservations(builder.footprint, this.window, counters)
        : null;
    this.loaded = builder.loaded == null ? new boolean[agents] : Arrays.copyOf(builder.loaded, agents);
    this.committedPrefixes = builder.committedPrefixes == null
        ? CommittedPrefix.copyOf(null, agents)
        : Arrays.copyOf(builder.committedPrefixes, agents);
    this.guidePaths = new int[agents][this.window];
    for (int[] path : this.guidePaths) {
      Arrays.fill(path, NO_VERTEX);
    }
    this.closed = new GuideNode[this.window][builder.roadmap.size() * STATE_SLOTS];
    this.guidedNext = new int[agents];
    Arrays.fill(this.guidedNext, NO_VERTEX);
    int heaviestStep = Roadmap.MAX_STEP_WORK;
    this.scaledCollisionCost = collisionCost * heaviestStep;
    this.scaledCollisionCostOrder = collisionCostOrder * heaviestStep;
    this.scaledFootprintCost = footprintCost * heaviestStep;
    this.cachedCollisionCosts = new float[agents];
    this.cachedFootprintConflicts = new int[agents];
    this.guideLogger = new LocalGuideLogger(
        builder.roadmap, builder.distTable, collisions, footprintReservations, builder.footprint,
        this.loaded, scaledCollisionCost, scaledCollisionCostOrder, scaledFootprintCost,
        this::heldByAnAgentThatHasNotPlannedYet);
    this.currentStep = 0;
  }

  public static final class Builder {

    private final Roadmap roadmap;
    private final DistTable distTable;
    private final int[] goals;
    private final int seed;
    private Footprint footprint;
    private boolean[] loaded;
    private CommittedPrefix[] committedPrefixes;
    private LegalMoves moves;
    private StaticBodies statics = StaticBodies.NONE;

    private Builder(Roadmap roadmap, DistTable distTable, int[] goals, int seed) {
      this.roadmap = roadmap;
      this.distTable = distTable;
      this.goals = goals;
      this.seed = seed;
      this.moves = new LegalMoves(roadmap);
    }

    public Builder footprint(Footprint footprint) {
      this.footprint = footprint;
      return this;
    }

    public Builder loaded(boolean[] loaded) {
      this.loaded = loaded;
      return this;
    }

    public Builder committedPrefixes(int[][] committedPrefixes) {
      this.committedPrefixes = CommittedPrefix.copyOf(committedPrefixes, goals.length);
      return this;
    }

    Builder committedPrefixes(CommittedPrefix[] committedPrefixes) {
      this.committedPrefixes = committedPrefixes;
      return this;
    }

    public Builder moves(LegalMoves moves) {
      this.moves = moves;
      return this;
    }

    public Builder statics(StaticBodies statics) {
      this.statics = statics;
      return this;
    }

    public FmsLocalGuide build() {
      return new FmsLocalGuide(this);
    }
  }

  public int window() {
    return window;
  }

  public boolean enabled() {
    return on;
  }

  public void construct(int[] configuration, int[] order) {
    construct(configuration, null, null, order, 0);
  }

  public void construct(int[] configuration, int[] noses, int[] order) {
    construct(configuration, noses, null, order, 0);
  }

  @Override
  public void construct(int[] configuration, int[] noses, int[] order, int solveDepth) {
    construct(configuration, noses, null, order, solveDepth);
  }

  public void construct(
      int[] configuration, int[] noses, int[] motions, int[] order, int solveDepth) {
    if (!on || numRefine < 0) {
      return;
    }

    boolean[] atGoal = null;
    if (clearGoalFirst) {
      atGoal = new boolean[agents];
      for (int agent = 0; agent < agents; agent++) {
        atGoal[agent] = configuration[agent] == goals[agent];
      }
    }
    standingAt = configuration;

    CommittedPrefix[] activePrefixes = activeCommittedPrefixes(configuration, solveDepth);
    boolean hasCommittedPrefix = Arrays.stream(activePrefixes).anyMatch(prefix -> prefix != null);
    int slidThisConstruct;
    if (hasCommittedPrefix) {
      clearAllEnrolledPaths();
      enrollCommittedPrefixes(activePrefixes, noses);
    }
    slidThisConstruct = carryGuidePathsForwardOneStep(configuration, noses, activePrefixes);
    int[][] carriedPaths = traceRolloutCarry ? currentGuidePaths() : null;
    if (counters.constructions == 0) {
      counters.warmStartSlides = slidThisConstruct;
      counters.warmFollowedItsCarriedPath = counters.followedItsCarriedPath;
      counters.warmAnchoredWithoutSpendingATick = counters.anchoredWithoutSpendingATick;
      counters.warmStandsOffItsCarriedPath = counters.standsOffItsCarriedPath;
      counters.warmHasNoCarriedPathAtAll = counters.hasNoCarriedPathAtAll;
    }
    counters.carryForwardSlides += slidThisConstruct;
    counters.constructions++;

    int refineIterations = numRefine == 0 ? 1 : numRefine;
    for (int round = 0; round < refineIterations; round++) {
      releaseAgentsOnTheirGoal(atGoal);
      for (int slot = 0; slot < agents; slot++) {
        int agent = order[slot];
        if (atGoal != null && atGoal[agent]) {
          continue;
        }
        if (numRefine == 0 && activePrefixes[agent] == null
            && guidePaths[agent][0] != guidePaths[agent][window - 1]) {
          continue;
        }
        guidedNext[agent] = NO_VERTEX;
        int[] pathBeforeRebuild = guideLogger.wantsRebuildChoice()
            ? guidePaths[agent].clone()
            : null;
        clear(agent);
        updateGuidePath(
            agent, configuration, noses, motions, round, slot, activePrefixes[agent],
            pathBeforeRebuild);
        enroll(agent, noseAtStart(noses, agent));
      }
      replanAgentsOnTheirGoal(atGoal, configuration, noses, motions, round, activePrefixes);
    }

    for (int agent = 0; agent < agents; agent++) {
      guidedNext[agent] = guidePaths[agent][1];
      clear(agent);
    }
    if (carriedPaths != null) {
      traceRebuiltCarriedPaths(solveDepth, configuration, activePrefixes, carriedPaths);
    }
  }

  private void traceRebuiltCarriedPaths(
      int solveDepth, int[] configuration, CommittedPrefix[] activePrefixes, int[][] carriedPaths) {
    for (int agent = 0; agent < agents; agent++) {
      int[] carried = carriedPaths[agent];
      int rebuiltNext = guidePaths[agent][1];
      boolean followedCarry = carried[0] == configuration[agent];
      if (!followedCarry || carried[1] == NO_VERTEX || carried[1] == rebuiltNext) {
        continue;
      }
      guideLogger.traceRolloutCarryRebuilt(
          counters.constructions, solveDepth, agent, configuration[agent],
          activePrefixes[agent] != null, carried, guidePaths[agent]);
    }
  }

  public int heuristic(int agent, int vertex) {
    if (!on || numRefine < 0 || guidedNext[agent] == NO_VERTEX) {
      return distTable.get(agent, vertex);
    }
    if (vertex == guidedNext[agent]) {
      return 0;
    }
    return distTable.get(agent, vertex) + 1;
  }

  public GuideCounters counters() {
    return counters;
  }

  public int[] guidedConfiguration() {
    return guidedNext;
  }

  public float collisionCostOf(int agent) {
    return cachedCollisionCosts[agent];
  }

  public int footprintConflictsOf(int agent) {
    return cachedFootprintConflicts[agent];
  }

  public void setGuidePaths(int[][] paths) {
    if (paths.length != agents) {
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
      if (copied > 0 && copied < window) {
        Arrays.fill(guidePaths[agent], copied, window, paths[agent][copied - 1]);
      }
      guidedNext[agent] = copied > 1 ? guidePaths[agent][1] : NO_VERTEX;
    }
  }

  public int[][] currentGuidePaths() {
    int[][] copy = new int[agents][];
    for (int agent = 0; agent < agents; agent++) {
      copy[agent] = guidePaths[agent].clone();
    }
    return copy;
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

  public int[][] pathsAtStep(int step) {
    return guidePathsHistory.get(step);
  }

  public void reconstructSolutionPaths(List<int[][]> solutionPaths) {
    guidePathsHistory.clear();
    guidePathsHistory.addAll(solutionPaths);
    currentStep = solutionPaths.size();
  }

  private CommittedPrefix[] activeCommittedPrefixes(int[] configuration, int solveDepth) {
    CommittedPrefix[] active = new CommittedPrefix[agents];
    for (int agent = 0; agent < agents; agent++) {
      CommittedPrefix fullPrefix = committedPrefixes[agent];
      if (fullPrefix.isActiveAt(solveDepth, configuration[agent])) {
        active[agent] = fullPrefix.remainingFrom(solveDepth, window);
      }
    }
    return active;
  }

  private void clearAllEnrolledPaths() {
    for (int agent = 0; agent < agents; agent++) {
      clear(agent);
    }
  }

  private void enrollCommittedPrefixes(CommittedPrefix[] prefixes, int[] noses) {
    for (int agent = 0; agent < agents; agent++) {
      CommittedPrefix prefix = prefixes[agent];
      if (prefix == null) {
        continue;
      }
      WarmStartAlignment alignment = alignGuidePathToCurrentCell(agent, standingAt[agent]);
      int[] warmStart = alignment == WarmStartAlignment.SLID
          || alignment == WarmStartAlignment.ANCHORED
          ? guidePaths[agent]
          : null;
      guidePaths[agent] = prefix.guidePathOver(warmStart, window);
      enroll(agent, noseAtStart(noses, agent));
    }
  }

  private void enrollRestingBody(int agent, int at, int startNose) {
    if (footprintReservations != null) {
      footprintReservations.enrollRestingBody(
          agent, at, startNose, loaded[agent], restingBodyTicks);
    }
  }

  private void enroll(int agent, int startNose) {
    collisions.enrollPath(agent, guidePaths[agent]);
    if (footprintReservations != null) {
      footprintReservations.enroll(agent, guidePaths[agent], startNose, loaded[agent]);
    }
  }

  private void clear(int agent) {
    collisions.clearPath(agent, guidePaths[agent]);
    if (footprintReservations != null) {
      footprintReservations.clear(agent);
    }
  }

  private int carryGuidePathsForwardOneStep(
      int[] configuration, int[] noses, CommittedPrefix[] activePrefixes) {
    int slid = 0;
    for (int agent = 0; agent < agents; agent++) {
      if (activePrefixes[agent] != null) {
        continue;
      }
      WarmStartAlignment alignment = alignGuidePathToCurrentCell(agent, configuration[agent]);
      if (alignment == WarmStartAlignment.SLID) {
        counters.followedItsCarriedPath++;
        slid++;
        enroll(agent, noseAtStart(noses, agent));
      } else {
        if (alignment == WarmStartAlignment.ANCHORED) {
          counters.anchoredWithoutSpendingATick++;
        } else if (alignment == WarmStartAlignment.NO_PATH) {
          counters.hasNoCarriedPathAtAll++;
        } else {
          counters.standsOffItsCarriedPath++;
          if (counters.constructions == 0) {
            guideLogger.reportStandingOffThePlan(agent, configuration[agent], guidePaths[agent]);
          }
        }
        enrollRestingBody(agent, configuration[agent], noseAtStart(noses, agent));
      }
    }
    return slid;
  }

  private WarmStartAlignment alignGuidePathToCurrentCell(int agent, int currentCell) {
    int[] path = guidePaths[agent];
    if (currentCell == path[1]) {
      for (int tick = 0; tick < window - 1; tick++) {
        path[tick] = path[tick + 1];
      }
      return WarmStartAlignment.SLID;
    }
    if (currentCell == path[0]) {
      return WarmStartAlignment.ANCHORED;
    }
    return path[0] == CollisionTable.NO_VERTEX
        ? WarmStartAlignment.NO_PATH
        : WarmStartAlignment.STANDS_OFF;
  }

  private void releaseAgentsOnTheirGoal(boolean[] atGoal) {
    if (atGoal == null) {
      return;
    }
    for (int agent = 0; agent < agents; agent++) {
      if (atGoal[agent]) {
        if (counters.constructions == 1) {
          counters.agentsReleasedBecauseTheyAreAtTheirGoal++;
        }
        collisions.clearPath(agent, guidePaths[agent]);
      }
    }
  }

  private void replanAgentsOnTheirGoal(
      boolean[] atGoal, int[] configuration, int[] noses, int[] motions, int refineRound,
      CommittedPrefix[] activePrefixes) {
    if (atGoal == null) {
      return;
    }
    for (int agent = 0; agent < agents; agent++) {
      if (!atGoal[agent]) {
        continue;
      }
      guidedNext[agent] = NO_VERTEX;
      int[] pathBeforeRebuild = guideLogger.wantsRebuildChoice()
          ? guidePaths[agent].clone()
          : null;
      clear(agent);
      updateGuidePath(
          agent, configuration, noses, motions, refineRound, -1, activePrefixes[agent],
          pathBeforeRebuild);
      enroll(agent, noseAtStart(noses, agent));
    }
  }

  private void updateGuidePath(
      int agent, int[] configuration, int[] noses, int[] motions, int refineRound, int orderSlot,
      CommittedPrefix committedPrefix, int[] pathBeforeRebuild) {
    int startWhen = committedPrefix == null ? 0 : committedPrefix.frontierTick();
    int start = committedPrefix == null ? configuration[agent] : committedPrefix.frontier();
    int startNose = committedPrefix == null
        ? noseAtStart(noses, agent)
        : committedPrefix.noseAfter(roadmap, noseAtStart(noses, agent));
    int fallbackMotion = motions == null ? Roadmap.DIR_NONE : motions[agent];
    int startMotion = committedPrefix == null
        ? fallbackMotion
        : committedPrefix.motionAfter(roadmap, fallbackMotion);
    int goal = goals[agent];
    int[] path = guidePaths[agent];

    if (committedPrefix == null) {
      Arrays.fill(path, NO_VERTEX);
    } else {
      committedPrefix.copyInto(path);
    }

    if (start == goal) {
      Arrays.fill(path, startWhen, window, start);
      cachedCollisionCosts[agent] = 0.0f;
      cachedFootprintConflicts[agent] = 0;
      return;
    }

    PriorityQueue<GuideNode> open = new PriorityQueue<>(CHEAPEST_THEN_STILLEST_THEN_DEEPEST);
    closedSlots.clear();
    open.add(nodeFor(agent, start, startNose, startMotion, null, startWhen));

    while (!open.isEmpty()) {
      GuideNode node = open.poll();
      int slot = closedSlotOf(node.where, node.nose, node.motion);
      if (closed[node.when][slot] != null) {
        continue;
      }
      closed[node.when][slot] = node;
      closedSlots.add(new int[] { node.when, slot });

      if (node.where == goal || node.when == window - 1) {
        cachedFootprintConflicts[agent] = node.footprintConflicts;
        cachedCollisionCosts[agent] = writeBack(agent, node, goal);
        guideLogger.traceSelectedBacktrackTail(agent, node, open.peek());
        break;
      }

      int[] candidates = moves.forAgent(agent, node.where, node.motion);
      if (!deterministic) {
        shuffle(candidates);
      }
      int nextWhen = node.when + 1;
      for (int candidate : candidates) {
        int candidateNose = roadmap.noseAfter(node.nose, node.where, candidate);
        int candidateMotion = roadmap.motionAfter(node.where, candidate);
        if (footprint != null
            && statics.hitBy(
                footprint,
                new Footprint.Motion(node.where, candidate, node.nose, loaded[agent]))) {
          continue;
        }
        int candidateSlot = closedSlotOf(candidate, candidateNose, candidateMotion);
        GuideNode alreadyClosed = closed[nextWhen][candidateSlot];
        if (alreadyClosed != null) {
          guideLogger.traceBacktrackCandidate(agent, node, null, alreadyClosed);
          continue;
        }
        GuideNode child = nodeFor(agent, candidate, candidateNose, candidateMotion, node, 0);
        guideLogger.traceBacktrackCandidate(agent, node, child, null);
        open.add(child);
      }
    }

    if (path[startWhen] == NO_VERTEX) {
      Arrays.fill(path, startWhen, window, start);
      cachedCollisionCosts[agent] = 0.0f;
      cachedFootprintConflicts[agent] = 0;
    }

    guideLogger.traceDetour(
        agent, path, noseAtStart(noses, agent), counters.constructions, refineRound, orderSlot);
    guideLogger.traceRevisitCounterfactual(
        agent, path, startNose, startWhen, refineRound, orderSlot);
    if (pathBeforeRebuild != null) {
      guideLogger.traceRebuildChoice(
          agent, counters.constructions, refineRound, orderSlot, start, startWhen, startNose,
          goal, pathBeforeRebuild, path);
    }

    for (int[] slot : closedSlots) {
      closed[slot[0]][slot[1]] = null;
    }
    closedSlots.clear();
  }

  private float writeBack(int agent, GuideNode tail, int goal) {
    int[] path = guidePaths[agent];
    float totalCollisionCost = 0.0f;
    for (GuideNode step = tail; step != null; step = step.parent) {
      path[step.when] = step.where;
      if (step.parent != null) {
        int collision = collisions.collisionCost(
            step.parent.where, step.where, step.parent.when);
        if (collision >= 1) {
          totalCollisionCost += scaledCollisionCost + collision * scaledCollisionCostOrder;
        }
      }
    }
    if (tail.where == goal) {
      for (int t = tail.when + 1; t < window; t++) {
        path[t] = goal;
      }
    }
    return totalCollisionCost;
  }

  private GuideNode nodeFor(
      int agent, int where, int nose, int motion, GuideNode parent, int startWhen) {
    GuideNode node = new GuideNode();
    node.when = parent == null ? startWhen : parent.when + 1;
    node.where = where;
    node.nose = nose;
    node.motion = motion;
    node.parent = parent;
    node.moves = parent == null ? 0 : parent.moves + (where == parent.where ? 0 : 1);
    int footprintConflict = parent == null || footprintReservations == null
        ? 0
        : footprintReservations.conflictsAt(
            new Footprint.Motion(
                parent.where, where, parent.nose, loaded[agent]),
            parent.when);
    node.footprintConflicts = parent == null
        ? 0
        : parent.footprintConflicts + footprintConflict;

    int collision = parent == null
        ? 0
        : collisions.collisionCost(parent.where, where, parent.when);
    counters.nodesExpanded++;
    float turnCost = parent == null ? 0.0f : Roadmap.turnCostBetween(parent.nose, nose);
    if (turnCost > 0) {
      counters.chargedForTurning++;
    }
    node.g = parent == null
        ? 0.0f
        : parent.g + roadmap.weightOf(where) + turnCost;
    if (collision >= 1) {
      counters.chargedForACellAnotherPlanAlreadyUses++;
      node.g += scaledCollisionCost + collision * scaledCollisionCostOrder;
    }
    if (footprintConflict >= 1) {
      counters.chargedForABodyInTheWay++;
      counters.bodiesInTheWayWhenCharged[Math.min(footprintConflict, 4)]++;
      node.g += scaledFootprintCost + footprintConflict * scaledCollisionCostOrder;
    }
    if (parent != null && heldByAnAgentThatHasNotPlannedYet(agent, where)) {
      counters.chargedForACellHeldByAnAgentThatHasNotPlannedYet++;
      node.g += scaledCollisionCost;
    }
    node.h = distTable.get(agent, where, nose);
    node.f = node.g + node.h;
    return node;
  }

  private boolean heldByAnAgentThatHasNotPlannedYet(int mover, int vertex) {
    if (standingAt == null || vertex == standingAt[mover]) {
      return false;
    }
    for (int other = 0; other < agents; other++) {
      if (other != mover
          && standingAt[other] == vertex
          && guidePaths[other][0] == NO_VERTEX) {
        return true;
      }
    }
    return false;
  }

  private int noseAtStart(int[] noses, int agent) {
    return noses == null ? Roadmap.DIR_NONE : noses[agent];
  }

  private static int closedSlotOf(int vertex, int nose, int motion) {
    return vertex * STATE_SLOTS + SLOT_BY_NOSE_AND_MOTION[stateIndex(nose, motion)];
  }

  private void shuffle(int[] values) {
    for (int i = values.length - 1; i > 0; i--) {
      int pick = (int) (rng.next() * (i + 1));
      int swapped = values[i];
      values[i] = values[pick];
      values[pick] = swapped;
    }
  }

  static final class GuideNode {

    int when;
    int where;
    int nose;
    int motion;
    int moves;
    int footprintConflicts;
    float g;
    float h;
    float f;
    GuideNode parent;

    GuideNode() {
    }
  }

  private enum WarmStartAlignment {
    SLID,
    ANCHORED,
    NO_PATH,
    STANDS_OFF
  }

}
