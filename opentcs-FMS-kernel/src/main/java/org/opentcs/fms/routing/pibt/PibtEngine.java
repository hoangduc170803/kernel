package org.opentcs.fms.routing.pibt;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import org.opentcs.fms.routing.diag.PibtCounters;
import org.opentcs.fms.routing.DistTable;
import org.opentcs.fms.routing.Guidance;
import org.opentcs.fms.routing.LegalMoves;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.StaticBodies;

public final class PibtEngine {

  private static final org.slf4j.Logger logger = org.slf4j.LoggerFactory.getLogger(PibtEngine.class);

  private static final int NO_AGENT = -1;
  static final int NO_VERTEX = -1;

  private static final boolean HINDRANCE = Boolean.parseBoolean(System.getProperty("fms.pibt.hindrance", "true"));

  public record Constraint(int agent, int vertex) {
  }

  private record CandidatePath(int[] vertices) {
    int source() {
      return vertices[0];
    }

    int destination() {
      return vertices[vertices.length - 1];
    }
  }

  private static final class CandidateTrace {

    private final int[] reasons;
    private final int[] blockers;
    private final int[] blockerTargets;

    CandidateTrace(int size) {
      reasons = new int[size];
      Arrays.fill(reasons, -1);
      blockers = new int[size];
      Arrays.fill(blockers, NO_AGENT);
      blockerTargets = new int[size];
      Arrays.fill(blockerTargets, NO_VERTEX);
    }
  }

  private final Roadmap roadmap;
  private final DistTable dist;
  private final int n;
  private final int v;
  private final int[] occNow;
  private final int[] occNext;
  private final int[] occTransition;
  private final int[] to;
  private final double[] tie;
  private final int[] candidateHindrance;
  private final Mulberry32 rng;

  private final boolean[] inChain;

  private final int[] decisionJournal;

  private final CandidatePath[] journalPaths;

  private int journalSize;

  private final int[] planAttempts;

  private final Footprint footprint;

  private final Guidance guide;

  private StaticBodies statics = StaticBodies.NONE;

  private final int[] noAxis;

  private final int[] noMotion;

  private final boolean[] noLoad;

  private int[] from;
  private int[] axis;
  private int[] motion;
  private boolean[] loaded;

  private static final int FAIL_CONSTRAINT_CELL_TAKEN = 0;
  private static final int FAIL_CONSTRAINT_UNREACHABLE = 1;
  private static final int FAIL_AGENT_OUT_OF_OPTIONS = 2;
  private static final int FAIL_BODIES_OVERLAP = 3;
  private static final int FAIL_CONSTRAINT_FOLLOWER = 4;

  private static final int SKIP_CELL_TAKEN = 0;
  private static final int SKIP_FOOTPRINT_VS_PLANNED = 1;
  private static final int SKIP_FOOTPRINT_VS_UNPLANNED = 2;
  private static final int SKIP_FOLLOWER = 3;

  private static final boolean TRACE_REFUSALS = Boolean
      .parseBoolean(System.getProperty("fms.pibt.traceRefusals", "true"));

  private static final boolean LOG_GUIDED_REFUSALS = Boolean.getBoolean("fms.pibt.logGuidedRefusals");

  private final StringBuilder[] skipTrace;

  private static final int MAX_GUIDED_REFUSAL_LINES = Integer.getInteger("fms.pibt.guidedRefusalLines", 20);

  private int guidedRefusalLines;
  private int fallbackLines;
  private static final int MAX_FALLBACK_LINES = 20;
  private static final int BLOCKER_STATIC = -2;

  private final LegalMoves moves;

  private final PibtCounters counters = new PibtCounters();

  private int blockerAgent = NO_AGENT;
  private int blockerTarget = NO_VERTEX;

  public static boolean hindranceEnabled() {
    return HINDRANCE;
  }

  public static Map<String, String> effectiveSettings() {
    return Map.of(
        "fms.pibt.hindrance", String.valueOf(HINDRANCE),
        "fms.pibt.traceRefusals", String.valueOf(TRACE_REFUSALS),
        "fms.pibt.logGuidedRefusals", String.valueOf(LOG_GUIDED_REFUSALS),
        "fms.pibt.guidedRefusalLines", Integer.toString(MAX_GUIDED_REFUSAL_LINES));
  }

  public PibtCounters counters() {
    return counters;
  }

  private void noteSkip(int agent, int vertex, int idx, int reason) {
    counters.skipped[reason]++;
    if (idx == 0) {
      counters.blockedFirstChoice[reason]++;
    }
    if (TRACE_REFUSALS && skipTrace[agent].length() < 600) {
      skipTrace[agent].append(' ').append(roadmap.nameOf(vertex))
          .append('=').append(PibtCounters.SKIP_REASONS[reason]);
      if (blockerAgent != NO_AGENT) {
        skipTrace[agent].append('(').append(blockerDetail()).append(')');
      }
    }
    if (guidanceActive() && guidedMiss(agent, vertex) == 0) {
      counters.guidedVertexRefused++;
      if (reason == SKIP_FOOTPRINT_VS_PLANNED || reason == SKIP_FOOTPRINT_VS_UNPLANNED) {
        if (guide.footprintConflictsOf(agent) == 0) {
          counters.guidedCellRefusedForAClashTheGuideNeverSaw++;
        } else {
          counters.guidedCellRefusedForAClashTheGuidePaidFor++;
        }
      }
      if (LOG_GUIDED_REFUSALS || guidedRefusalLines < MAX_GUIDED_REFUSAL_LINES) {
        guidedRefusalLines++;
        logger.info(
            "[PIBT guided-refused] a{}@{} wanted {} at rank {} but it was skipped: {} {}",
            agent, roadmap.nameOf(from[agent]), roadmap.nameOf(vertex), idx,
            PibtCounters.SKIP_REASONS[reason], blockerDetail());
      }
    }
    blockerAgent = NO_AGENT;
  }

  private int[] fail(int cause, int agent) {
    counters.failures[cause]++;
    if (counters.firstFailureCause < 0) {
      counters.firstFailureCause = cause;
      counters.firstFailureDetail = agent < 0 ? "" : describeStarvedAgent(agent);
    }
    cleanupOcc(from);
    return null;
  }

  private String describeStarvedAgent(int agent) {
    int guided = guidanceActive() ? guide.guidedConfiguration()[agent] : NO_VERTEX;
    return "a" + agent + "@" + roadmap.nameOf(from[agent])
        + " loaded=" + loaded[agent]
        + " guideWanted=" + (guided == NO_VERTEX ? "none" : roadmap.nameOf(guided))
        + " rejected:" + skipTrace[agent];
  }

  private boolean movesAwayFromGoal(int agent, int vnow, int chosen) {
    return dist.get(agent, chosen, noseArrivingAt(agent, chosen)) > dist.get(agent, vnow, axis[agent]);
  }

  public PibtEngine(Roadmap roadmap, int numAgents, int seed, DistTable dist) {
    this(roadmap, numAgents, seed, dist, null);
  }

  public PibtEngine(
      Roadmap roadmap, int numAgents, int seed, DistTable dist, Guidance guide) {
    this(roadmap, numAgents, seed, dist, guide,
        Footprint.enabled() ? new Footprint(roadmap) : null);
  }

  public PibtEngine(
      Roadmap roadmap, int numAgents, int seed, DistTable dist, Guidance guide,
      Footprint footprint) {
    this(roadmap, numAgents, seed, dist, guide, footprint, new LegalMoves(roadmap));
  }

  public PibtEngine(
      Roadmap roadmap, int numAgents, int seed, DistTable dist, Guidance guide,
      Footprint footprint, LegalMoves moves) {
    this.moves = moves;
    this.footprint = footprint;
    this.guide = guide;
    this.roadmap = roadmap;
    this.dist = dist;
    this.n = numAgents;
    this.v = roadmap.size();
    this.occNow = new int[v];
    this.occNext = new int[v];
    this.occTransition = new int[v];
    Arrays.fill(this.occNow, NO_AGENT);
    Arrays.fill(this.occTransition, NO_AGENT);
    this.to = new int[n];
    this.tie = new double[v];
    this.candidateHindrance = new int[v];
    this.inChain = new boolean[n];
    this.decisionJournal = new int[n];
    this.journalPaths = new CandidatePath[n];
    this.planAttempts = new int[n];
    this.rng = new Mulberry32(seed);
    this.noAxis = new int[n];
    Arrays.fill(this.noAxis, Roadmap.DIR_NONE);
    this.noMotion = new int[n];
    Arrays.fill(this.noMotion, Roadmap.DIR_NONE);
    this.noLoad = new boolean[n];
    this.skipTrace = new StringBuilder[n];
    for (int a = 0; a < n; a++) {
      this.skipTrace[a] = new StringBuilder();
    }
  }

  public int[] step(int[] from, int[] order, List<Constraint> constraints) {
    return step(from, order, constraints, null, null, StaticBodies.NONE, null);
  }

  public int[] step(
      int[] from, int[] order, List<Constraint> constraints, int[] axis,
      boolean[] loaded, StaticBodies statics, int[] motion) {
    this.motion = motion == null ? noMotion : motion;
    this.statics = statics == null ? StaticBodies.NONE : statics;
    this.from = from;
    this.axis = axis == null ? noAxis : axis;
    this.loaded = loaded == null ? noLoad : loaded;
    Arrays.fill(occNext, NO_AGENT);
    Arrays.fill(occTransition, NO_AGENT);
    Arrays.fill(to, NO_VERTEX);
    Arrays.fill(planAttempts, 0);
    Arrays.fill(journalPaths, null);
    journalSize = 0;
    for (int a = 0; a < n; a++) {
      occNow[from[a]] = a;
    }

    for (Constraint c : constraints) {
      int vtx = c.vertex();
      if (occNext[vtx] != NO_AGENT) {
        return fail(FAIL_CONSTRAINT_CELL_TAKEN, -1);
      }
      if (!isReachable(from[c.agent()], vtx)) {
        return fail(FAIL_CONSTRAINT_UNREACHABLE, -1);
      }
      int occ = occNow[vtx];
      if (occ != NO_AGENT && occ != c.agent()) {
        return fail(FAIL_CONSTRAINT_FOLLOWER, -1);
      }
      CandidatePath path = new CandidatePath(new int[] { from[c.agent()], vtx });
      if (pathTakenByOther(c.agent(), path)) {
        return fail(FAIL_CONSTRAINT_FOLLOWER, -1);
      }
      occNext[vtx] = c.agent();
      occupyPath(c.agent(), path);
      to[c.agent()] = vtx;
    }

    for (int a = 0; a < n; a++) {
      for (int u : roadmap.candidates(from[a])) {
        tie[u] = rng.next();
      }
    }

    for (int a : order) {
      if (to[a] == NO_VERTEX && !funcPIBT(a)) {
        return fail(FAIL_AGENT_OUT_OF_OPTIONS, a);
      }
    }

    int[] next = new int[n];
    System.arraycopy(to, 0, next, 0, n);
    if (bodiesOverlap(next)) {
      return fail(FAIL_BODIES_OVERLAP, -1);
    }
    cleanupOcc(from);
    return next;
  }

  private boolean bodiesOverlap(int[] next) {
    if (footprint == null) {
      return false;
    }
    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        if (!footprint.mayInteract(from[i], from[j])) {
          continue;
        }
        if (footprint.collideMoving(motionOf(i, next[i]), motionOf(j, next[j]))) {
          counters.overlappingConfigurations++;
          return true;
        }
      }
      if (hitsAStaticBody(motionOf(i, next[i]))) {
        counters.overlappingConfigurations++;
        return true;
      }
    }
    return false;
  }

  private boolean hitsAStaticBody(Footprint.Motion mine) {
    return statics.hitBy(footprint, mine);
  }

  private boolean clearsPlannedMotions(int i, int vnow, int u) {
    if (footprint == null) {
      return true;
    }
    if (footprint.isRotation(axis[i], vnow, u)) {
      counters.turnsConsidered++;
    }
    for (int j = 0; j < n; j++) {
      if (j == i || to[j] == NO_VERTEX || !footprint.mayInteract(vnow, from[j])) {
        continue;
      }
      if (footprint.collideMoving(movingTo(i, vnow, u), motionOf(j, to[j]))) {
        counters.footprintRejections++;
        noteBlocker(j, to[j]);
        return false;
      }
    }
    if (hitsAStaticBody(movingTo(i, vnow, u))) {
      counters.footprintRejections++;
      blockerAgent = BLOCKER_STATIC;
      return false;
    }
    return true;
  }

  private void noteBlocker(int j, int target) {
    blockerAgent = j;
    blockerTarget = target;
  }

  private String blockerDetail() {
    if (blockerAgent == BLOCKER_STATIC) {
      return "staticBody";
    }
    if (blockerAgent == NO_AGENT) {
      return "";
    }
    Footprint.Motion theirs = motionOf(blockerAgent, blockerTarget);
    return "a" + blockerAgent + "@" + roadmap.nameOf(from[blockerAgent])
        + "->" + roadmap.nameOf(blockerTarget)
        + (footprint != null && theirs.turns(footprint) ? " rotating" : " straight")
        + (loaded[blockerAgent] ? " loaded" : " empty");
  }

  private boolean clearsUnplannedMotions(int i, int vnow, int u) {
    if (footprint == null) {
      return true;
    }
    for (int j = 0; j < n; j++) {
      if (j == i || to[j] != NO_VERTEX || !footprint.mayInteract(vnow, from[j])) {
        continue;
      }
      if (!footprint.collideMoving(movingTo(i, vnow, u), restingOf(j))) {
        continue;
      }
      if (inChain[j]) {
        counters.footprintRejections++;
        noteBlocker(j, from[j]);
        return false;
      }
      counters.footprintPushes++;
      if (!funcPIBT(j)) {
        counters.footprintRejections++;
        noteBlocker(j, from[j]);
        return false;
      }
    }
    return true;
  }

  private void cleanupOcc(int[] from) {
    for (int a = 0; a < n; a++) {
      occNow[from[a]] = NO_AGENT;
    }
  }

  private boolean isReachable(int fromV, int toV) {
    if (fromV == toV) {
      return true;
    }
    for (int nb : roadmap.neighborsOf(fromV)) {
      if (nb == toV) {
        return true;
      }
    }
    return false;
  }

  private boolean funcPIBT(int i) {
    if (planAttempts[i]++ > 0) {
      counters.replans++;
    }
    inChain[i] = true;
    skipTrace[i].setLength(0);
    try {
      return decideNextPath(i);
    } finally {
      inChain[i] = false;
    }
  }

  private int noseArrivingAt(int agent, int vertex) {
    return vertex == from[agent] ? axis[agent] : roadmap.requiredNose(from[agent], vertex);
  }

  private int costThrough(int agent, int vertex) {
    int nose = noseArrivingAt(agent, vertex);
    return roadmap.weightOf(vertex)
        + Roadmap.turnCostBetween(axis[agent], nose)
        + dist.get(agent, vertex, nose);
  }

  private int hindranceOf(int agent, int vertex) {
    int hindered = 0;
    for (int around : roadmap.neighborsOf(from[agent])) {
      int other = occNow[around];
      if (other == NO_AGENT || other == agent || from[other] == vertex) {
        continue;
      }
      if (dist.get(other, vertex) < dist.get(other, from[other])) {
        hindered++;
      }
    }
    return hindered;
  }

  private boolean guidanceActive() {
    return guide != null && guide.enabled();
  }

  private int guidedMiss(int agent, int vertex) {
    int guided = guide.guidedConfiguration()[agent];
    return guided == NO_VERTEX || guided == vertex ? 0 : 1;
  }

  private Footprint.Motion motionOf(int agent, int destination) {
    return new Footprint.Motion(
        from[agent], destination, axis[agent], loaded[agent]);
  }

  private Footprint.Motion restingOf(int agent) {
    return Footprint.Motion.resting(from[agent], axis[agent], loaded[agent]);
  }

  private Footprint.Motion movingTo(int agent, int origin, int destination) {
    return new Footprint.Motion(
        origin, destination, axis[agent], loaded[agent]);
  }

  private int compareCandidates(int agent, int x, int y) {
    if (guidanceActive()) {
      int byGuidance = Integer.compare(guidedMiss(agent, x), guidedMiss(agent, y));
      if (byGuidance != 0) {
        counters.guidanceDecisions++;
        return byGuidance;
      }
    }
    int byCost = Integer.compare(costThrough(agent, x), costThrough(agent, y));
    if (byCost != 0) {
      return byCost;
    }
    int byHindrance = Integer.compare(candidateHindrance[x], candidateHindrance[y]);
    if (byHindrance != 0) {
      counters.hindranceDecisions++;
      return byHindrance;
    }
    return Double.compare(tie[x], tie[y]);
  }

  private boolean decideNextPath(int i) {
    int vnow = from[i];
    CandidateTrace trace = null;

    int[] candArr = moves.forAgent(i, vnow, motion[i]);
    CandidatePath[] candidates = new CandidatePath[candArr.length];
    for (int k = 0; k < candArr.length; k++) {
      int destination = candArr[k];
      candidates[k] = new CandidatePath(new int[] { vnow, destination });
      candidateHindrance[destination] = HINDRANCE ? hindranceOf(i, destination) : 0;
    }
    Arrays.sort(candidates, (x, y) -> compareCandidates(i, x.destination(), y.destination()));

    for (int idx = 0; idx < candidates.length; idx++) {
      CandidatePath path = candidates[idx];
      int u = path.destination();
      if (occNext[u] != NO_AGENT) {
        trace = noteCandidateSkip(i, u, idx, SKIP_CELL_TAKEN, candidates.length, trace);
        continue;
      }
      int follower = followerBlocker(i, path);
      if (follower != NO_AGENT) {
        noteBlocker(follower, from[follower]);
        trace = noteCandidateSkip(i, u, idx, SKIP_FOLLOWER, candidates.length, trace);
        continue;
      }
      if (pathTakenByOther(i, path)) {
        trace = noteCandidateSkip(i, u, idx, SKIP_FOLLOWER, candidates.length, trace);
        continue;
      }
      if (!clearsPlannedMotions(i, vnow, u)) {
        trace = noteCandidateSkip(
            i, u, idx, SKIP_FOOTPRINT_VS_PLANNED, candidates.length, trace);
        continue;
      }
      int mark = journalSize;
      occNext[u] = i;
      occupyPath(i, path);
      to[i] = u;
      recordDecision(i, path);
      if (!clearsUnplannedMotions(i, vnow, u)) {
        unwindTo(mark);
        trace = noteCandidateSkip(
            i, u, idx, SKIP_FOOTPRINT_VS_UNPLANNED, candidates.length, trace);
        continue;
      }
      if (u == vnow) {
        counters.choseToStay++;
      } else if (movesAwayFromGoal(i, vnow, u)) {
        counters.stepsAwayFromGoal++;
      }
      if (idx > 0 && trace != null && fallbackLines < MAX_FALLBACK_LINES
          && movesAwayFromGoal(i, vnow, u)) {
        logFallback(i, vnow, u, idx, candidates, trace);
      }
      return true;
    }

    counters.stepsHeldInPlace++;
    counters.agentsLeftOpen++;
    return false;
  }

  private CandidateTrace noteCandidateSkip(
      int agent, int vertex, int rank, int reason, int candidateCount, CandidateTrace trace) {
    int blocker = blockerAgent;
    int blockerTarget = this.blockerTarget;
    noteSkip(agent, vertex, rank, reason);
    if (trace == null && rank == 0 && logger.isInfoEnabled()) {
      trace = new CandidateTrace(candidateCount);
    }
    if (trace != null) {
      trace.reasons[rank] = reason;
      trace.blockers[rank] = blocker;
      trace.blockerTargets[rank] = blockerTarget;
    }
    return trace;
  }

  private void logFallback(
      int agent, int current, int chosen, int chosenRank, CandidatePath[] candidates,
      CandidateTrace trace) {
    StringBuilder details = new StringBuilder();
    for (int rank = 0; rank < candidates.length; rank++) {
      if (rank > 0) {
        details.append(" | ");
      }
      int vertex = candidates[rank].destination();
      details.append('#').append(rank).append(' ').append(roadmap.nameOf(vertex));
      if (rank == chosenRank) {
        details.append(" CHOSE");
      } else if (trace.reasons[rank] >= 0) {
        details.append(" REFUSED(")
            .append(PibtCounters.SKIP_REASONS[trace.reasons[rank]]);
        int blocker = trace.blockers[rank];
        if (blocker == BLOCKER_STATIC) {
          details.append(" staticBody");
        } else if (blocker >= 0) {
          details.append(" a").append(blocker).append('@')
              .append(roadmap.nameOf(from[blocker]));
          int target = trace.blockerTargets[rank];
          if (target != NO_VERTEX) {
            details.append("->").append(roadmap.nameOf(target));
          }
        }
        details.append(')');
      } else if (vertex == current) {
        details.append(" STAY");
      } else {
        details.append(" UNTESTED");
      }
      details.append(" d=").append(dist.get(agent, vertex, noseArrivingAt(agent, vertex)));
    }
    fallbackLines++;
    logger.info("[PIBT fallback] a{}@{} nose={} {}: {}", agent, roadmap.nameOf(current),
        axisName(axis[agent]), loaded[agent] ? "loaded" : "empty", details);
  }

  private static String axisName(int axis) {
    return switch (axis) {
      case Roadmap.AXIS_X -> "X";
      case Roadmap.AXIS_Y -> "Y";
      default -> "none";
    };
  }

  private void recordDecision(int agent, CandidatePath path) {
    decisionJournal[journalSize] = agent;
    journalPaths[journalSize] = path;
    journalSize++;
  }

  private void unwindTo(int mark) {
    counters.abandonedPushes += journalSize - mark - 1;
    while (journalSize > mark) {
      journalSize--;
      int agent = decisionJournal[journalSize];
      CandidatePath path = journalPaths[journalSize];
      journalPaths[journalSize] = null;
      occNext[to[agent]] = NO_AGENT;
      to[agent] = NO_VERTEX;
      releasePath(agent, path);
    }
  }

  private int followerBlocker(int agent, CandidatePath path) {
    for (int k = 1; k < path.vertices().length; k++) {
      int occupant = occNow[path.vertices()[k]];
      if (occupant != NO_AGENT && occupant != agent) {
        return occupant;
      }
    }
    return NO_AGENT;
  }

  private boolean pathTakenByOther(int agent, CandidatePath path) {
    for (int vertex : path.vertices()) {
      if (occTransition[vertex] != NO_AGENT && occTransition[vertex] != agent) {
        return true;
      }
    }
    return false;
  }

  private void occupyPath(int agent, CandidatePath path) {
    for (int vertex : path.vertices()) {
      occTransition[vertex] = agent;
    }
  }

  private void releasePath(int agent, CandidatePath path) {
    for (int vertex : path.vertices()) {
      if (occTransition[vertex] == agent) {
        occTransition[vertex] = NO_AGENT;
      }
    }
  }
}
