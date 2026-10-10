package org.opentcs.fms.routing;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.BiPredicate;

import org.opentcs.fms.routing.diag.SolveReport;
import org.opentcs.fms.routing.pibt.Footprint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Diagnostic rendering for {@link FmsLocalGuide}.
 *
 * <p>
 * This class deliberately has no search or reservation mutation
 * responsibilities. It reads the
 * same tables as the guide to render the score breakdown that led to a
 * diagnostic message.
 */
final class LocalGuideLogger {

  private static final Logger logger = LoggerFactory.getLogger(LocalGuideLogger.class);

  private static final boolean TRACE_BACKTRACK = Boolean
      .parseBoolean(System.getProperty("fms.guide.traceBacktrack", "false"));

  private static final boolean TRACE_COUNTERFACTUAL = Boolean
      .parseBoolean(System.getProperty("fms.guide.traceCounterfactual", "false"));

  private static final boolean TRACE_DETOUR = Boolean
      .parseBoolean(System.getProperty("fms.guide.traceDetour", "false"));

  private static final boolean TRACE_ROLLOUT_CARRY_REBUILT = Boolean
      .parseBoolean(System.getProperty("fms.guide.traceRolloutCarryRebuilt", "false"));

  private static final boolean TRACE_REBUILD_CHOICE = Boolean
      .parseBoolean(System.getProperty("fms.guide.traceRebuildChoice", "false"));

  private static final int MAX_BACKTRACK_CANDIDATE_LINES =
      Integer.getInteger("fms.guide.backtrackCandidateLines", 3);

  private static final int MAX_EXCURSION_TICKS =
      Integer.getInteger("fms.guide.excursionTicks", 12);

  private static final int MAX_BACKTRACK_TAIL_LINES =
      Integer.getInteger("fms.guide.backtrackTailLines", 20);

  private static final int MAX_COUNTERFACTUAL_LINES =
      Integer.getInteger("fms.guide.counterfactualLines", 300);

  private static final int MAX_REBUILD_CHOICE_LINES =
      Integer.getInteger("fms.guide.rebuildChoiceLines", 120);

  private static final int MAX_REBUILD_CHOICE_STEPS =
      Integer.getInteger("fms.guide.rebuildChoiceSteps", 8);

  private int counterfactualLines;

  private int rebuildChoiceLines;

  private int backtrackCandidateLines;

  private int backtrackTailLines;

  private final Roadmap roadmap;
  private final DistTable distTable;
  private final CollisionTable collisions;
  private final GuideFootprintReservations footprintReservations;
  private final Footprint footprint;
  private final boolean[] loaded;
  private final float scaledCollisionCost;
  private final float scaledCollisionCostOrder;
  private final float scaledFootprintCost;
  private final BiPredicate<Integer, Integer> heldByUnplannedAgent;

  LocalGuideLogger(
      Roadmap roadmap, DistTable distTable, CollisionTable collisions,
      GuideFootprintReservations footprintReservations, Footprint footprint, boolean[] loaded,
      float scaledCollisionCost, float scaledCollisionCostOrder, float scaledFootprintCost,
      BiPredicate<Integer, Integer> heldByUnplannedAgent) {
    this.roadmap = roadmap;
    this.distTable = distTable;
    this.collisions = collisions;
    this.footprintReservations = footprintReservations;
    this.footprint = footprint;
    this.loaded = loaded;
    this.scaledCollisionCost = scaledCollisionCost;
    this.scaledCollisionCostOrder = scaledCollisionCostOrder;
    this.scaledFootprintCost = scaledFootprintCost;
    this.heldByUnplannedAgent = heldByUnplannedAgent;
  }

  void traceRevisitCounterfactual(
      int agent, int[] path, int startNose, int startWhen, int refineRound, int orderSlot) {
    if (!TRACE_COUNTERFACTUAL || !logger.isInfoEnabled()
        || counterfactualLines >= MAX_COUNTERFACTUAL_LINES) {
      return;
    }
    int nose = startNose;
    for (int t = startWhen; t + 2 < path.length; t++) {
      int here = path[t];
      int out = path[t + 1];
      int back = path[t + 2];
      if (here == CollisionTable.NO_VERTEX || out == CollisionTable.NO_VERTEX
          || back == CollisionTable.NO_VERTEX) {
        return;
      }
      if (here == back && here != out) {
        emitCounterfactual(agent, here, out, nose, t, refineRound, orderSlot);
        return;
      }
      nose = roadmap.noseAfter(nose, here, out);
    }
  }

  private void emitCounterfactual(
      int agent, int here, int out, int nose, int time, int refineRound, int orderSlot) {
    counterfactualLines++;
    int noseOut = roadmap.noseAfter(nose, here, out);
    int noseBack = roadmap.noseAfter(noseOut, out, here);
    logger.info(
        "[MAPF guide-cf] a{} refine={} slot={} revisit={}@t{} -> {}@t{} -> {}@t{}",
        agent, refineRound, orderSlot, roadmap.nameOf(here), time, roadmap.nameOf(out), time + 1,
        roadmap.nameOf(here), time + 2);
    float diveOut = describeCounterfactualStep(
        agent, "DIVE out ", here, out, nose, time);
    float diveBack = describeCounterfactualStep(
        agent, "DIVE back", out, here, noseOut, time + 1);
    float stayOne = describeCounterfactualStep(
        agent, "STAY hold", here, here, nose, time);
    float stayTwo = describeCounterfactualStep(
        agent, "STAY hold", here, here, nose, time + 1);
    logger.info(
        "[MAPF guide-cf] a{} totals dive dg={} h@t{}={} | stay dg={} h@t{}={}",
        agent, diveOut + diveBack, time + 2, distTable.get(agent, here, noseBack),
        stayOne + stayTwo, time + 2, distTable.get(agent, here, nose));
  }

  private float describeCounterfactualStep(
      int agent, String label, int from, int to, int noseBefore, int time) {
    int noseAfter = roadmap.noseAfter(noseBefore, from, to);
    int weight = roadmap.weightOf(to);
    int collision = collisions.collisionCost(from, to, time);
    int bodies = footprintReservations == null
        ? 0
        : footprintReservations.conflictsAtQuietly(
            new Footprint.Motion(from, to, noseBefore, loaded[agent]), time);
    float turn = Roadmap.turnCostBetween(noseBefore, noseAfter);
    boolean unplanned = heldByUnplannedAgent.test(agent, to);
    float g = weight + turn
        + (collision >= 1 ? scaledCollisionCost + collision * scaledCollisionCostOrder : 0.0f)
        + (bodies >= 1 ? scaledFootprintCost + bodies * scaledCollisionCostOrder : 0.0f)
        + (unplanned ? scaledCollisionCost : 0.0f);
    logger.info(
        "[MAPF guide-cf] a{} {} {}->{} t{} w={} turn={} coll={} fp={} unplanned={}"
            + " h={} dg={}",
        agent, label, roadmap.nameOf(from), roadmap.nameOf(to), time, weight, turn,
        collision, bodies, unplanned ? "y" : "n",
        distTable.get(agent, to, noseAfter), g);
    return g;
  }

  void reportStandingOffThePlan(int agent, int where, int[] carriedPath) {
    logger.debug(
        "[MAPF off-plan] a{} stands on {} but its carried path starts [{}] and holds that cell"
            + " {} -- the seed it was handed no longer describes where it is",
        agent, roadmap.nameOf(where),
        SolveReport.carriedPathHead(roadmap, carriedPath, CollisionTable.NO_VERTEX),
        SolveReport.whereTheCarriedPathHolds(carriedPath, where));
  }

  /** Emits the facts needed to compare an A-...-A excursion with waiting at A as many ticks. */
  void traceBacktrackCandidate(
      int agent, FmsLocalGuide.GuideNode pivot, FmsLocalGuide.GuideNode returned,
      FmsLocalGuide.GuideNode alreadyClosed) {
    if (!TRACE_BACKTRACK || !logger.isInfoEnabled() || pivot.parent == null
        || (returned == null && alreadyClosed == null)) {
      return;
    }
    int candidate = returned == null ? alreadyClosed.where : returned.where;
    FmsLocalGuide.GuideNode anchor = excursionAnchor(pivot, candidate);
    if (anchor == null) {
      return;
    }
    if (backtrackCandidateLines >= MAX_BACKTRACK_CANDIDATE_LINES) {
      noteCap(backtrackCandidateLines++, MAX_BACKTRACK_CANDIDATE_LINES,
          "candidate", "fms.guide.backtrackCandidateLines");
      return;
    }
    backtrackCandidateLines++;

    List<FmsLocalGuide.GuideNode> loop = chainFrom(anchor, pivot);
    int ticks = pivot.when + 1 - anchor.when;
    int candidateNose = roadmap.noseAfter(pivot.nose, pivot.where, candidate);
    logger.info(
        "[MAPF guide-backtrack] a{} anchor {}@t{} nose={} returns at t{} after {} ticks via {}"
            + " | {}",
        agent, roadmap.nameOf(anchor.where), anchor.when, noseName(anchor.nose), pivot.when + 1,
        ticks, excursionCells(loop, candidate),
        returned == null
            ? "CLOSED by " + roadmap.nameOf(alreadyClosed.where) + "@t" + alreadyClosed.when
                + " nose=" + noseName(alreadyClosed.nose)
            : "enqueued nose=" + noseName(returned.nose));

    StringBuilder excursionSteps = new StringBuilder();
    float excursionG = anchor.g;
    for (int index = 1; index < loop.size(); index++) {
      excursionG += describeMove(excursionSteps, agent, loop.get(index - 1), loop.get(index).where);
    }
    excursionG += describeMove(excursionSteps, agent, pivot, candidate);
    emitTerminal(agent, "EXCURSION", excursionG,
        distTable.get(agent, candidate, candidateNose), ticks, excursionSteps,
        returned == null ? null : returned.f);

    StringBuilder waitSteps = new StringBuilder();
    float waitG = anchor.g;
    for (int tick = 0; tick < ticks; tick++) {
      waitG += describeStep(waitSteps, agent, anchor.where, anchor.where, anchor.nose,
          anchor.nose, anchor.when + tick);
    }
    emitTerminal(agent, "WAIT     ", waitG,
        distTable.get(agent, anchor.where, anchor.nose), ticks, waitSteps, null);

    emitDirect(agent, anchor);
  }

  private void emitDirect(int agent, FmsLocalGuide.GuideNode anchor) {
    int best = CollisionTable.NO_VERTEX;
    float bestF = Float.MAX_VALUE;
    for (int neighbour : roadmap.candidates(anchor.where)) {
      if (neighbour == anchor.where) {
        continue;
      }
      float f = anchor.g + moveCostQuietly(agent, anchor, neighbour)
          + distTable.get(agent, neighbour,
              roadmap.noseAfter(anchor.nose, anchor.where, neighbour));
      if (f < bestF) {
        bestF = f;
        best = neighbour;
      }
    }
    if (best == CollisionTable.NO_VERTEX) {
      return;
    }
    StringBuilder directSteps = new StringBuilder();
    float directG = anchor.g + describeMove(directSteps, agent, anchor, best);
    emitTerminal(agent, "DIRECT   ", directG,
        distTable.get(agent, best, roadmap.noseAfter(anchor.nose, anchor.where, best)),
        1, directSteps, null);
  }

  private void emitTerminal(
      int agent, String label, float g, float h, int ticks, StringBuilder steps, Float searchF) {
    logger.info(
        "[MAPF guide-backtrack] a{} {} ticks={} g={} h={} f={}{} | {}",
        agent, label, ticks, g, h, g + h,
        searchF == null ? "" : " searchF=" + searchF, steps);
  }

  private FmsLocalGuide.GuideNode excursionAnchor(FmsLocalGuide.GuideNode pivot, int candidate) {
    boolean leftTheCell = false;
    for (FmsLocalGuide.GuideNode step = pivot; step != null; step = step.parent) {
      if (pivot.when - step.when > MAX_EXCURSION_TICKS) {
        return null;
      }
      if (step.where == candidate) {
        return leftTheCell ? step : null;
      }
      leftTheCell = true;
    }
    return null;
  }

  private static List<FmsLocalGuide.GuideNode> chainFrom(
      FmsLocalGuide.GuideNode anchor, FmsLocalGuide.GuideNode pivot) {
    List<FmsLocalGuide.GuideNode> chain = new ArrayList<>();
    for (FmsLocalGuide.GuideNode step = pivot; step != null; step = step.parent) {
      chain.add(step);
      if (step == anchor) {
        break;
      }
    }
    Collections.reverse(chain);
    return chain;
  }

  private String excursionCells(List<FmsLocalGuide.GuideNode> loop, int candidate) {
    StringBuilder cells = new StringBuilder();
    for (FmsLocalGuide.GuideNode step : loop) {
      cells.append(roadmap.nameOf(step.where)).append("->");
    }
    return cells.append(roadmap.nameOf(candidate)).toString();
  }

  private float describeMove(
      StringBuilder out, int agent, FmsLocalGuide.GuideNode parent, int to) {
    int noseTo = roadmap.noseAfter(parent.nose, parent.where, to);
    return describeStep(out, agent, parent.where, to, parent.nose, noseTo, parent.when);
  }

  private float moveCostQuietly(int agent, FmsLocalGuide.GuideNode parent, int to) {
    int noseTo = roadmap.noseAfter(parent.nose, parent.where, to);
    float turn = Roadmap.turnCostBetween(parent.nose, noseTo);
    int collision = collisions.collisionCost(parent.where, to, parent.when);
    int bodies = footprintReservations == null
        ? 0
        : footprintReservations.conflictsAtQuietly(
            new Footprint.Motion(parent.where, to, parent.nose, loaded[agent]), parent.when);
    return roadmap.weightOf(to) + turn
        + (collision >= 1 ? scaledCollisionCost + collision * scaledCollisionCostOrder : 0.0f)
        + (bodies >= 1 ? scaledFootprintCost + bodies * scaledCollisionCostOrder : 0.0f)
        + (heldByUnplannedAgent.test(agent, to) ? scaledCollisionCost : 0.0f);
  }

  void traceSelectedBacktrackTail(
      int agent, FmsLocalGuide.GuideNode tail, FmsLocalGuide.GuideNode openPeek) {
    if (!TRACE_BACKTRACK || !logger.isInfoEnabled() || !hasImmediateBacktrack(tail)) {
      return;
    }
    if (backtrackTailLines >= MAX_BACKTRACK_TAIL_LINES) {
      noteCap(backtrackTailLines++, MAX_BACKTRACK_TAIL_LINES,
          "SELECTED tail", "fms.guide.backtrackTailLines");
      return;
    }
    backtrackTailLines++;
    logger.info(
        "[MAPF guide-backtrack] a{} SELECTED tail {}@t{} nose={} g={} h={} f={} moves={}"
            + " | openPeek={}",
        agent, roadmap.nameOf(tail.where), tail.when, noseName(tail.nose), tail.g, tail.h, tail.f,
        tail.moves, openPeek == null ? "none" : describeNode(openPeek));
  }

  void traceDetour(
      int agent, int[] path, int startNose, int constructions, int refineRound, int orderSlot) {
    if (!TRACE_DETOUR || !logger.isInfoEnabled()) {
      return;
    }
    int pivot = -1;
    for (int t = 1; t + 1 < path.length; t++) {
      if (path[t] != CollisionTable.NO_VERTEX && path[t + 1] != CollisionTable.NO_VERTEX
          && path[t - 1] == path[t + 1] && path[t] != path[t - 1]) {
        pivot = t;
        break;
      }
    }
    if (pivot < 0) {
      return;
    }
    int[] noses = new int[path.length];
    noses[0] = startNose;
    for (int t = 1; t < path.length && path[t] != CollisionTable.NO_VERTEX; t++) {
      noses[t] = roadmap.noseAfter(noses[t - 1], path[t - 1], path[t]);
    }
    int anchor = path[pivot - 1];
    StringBuilder took = new StringBuilder();
    float tookTotal = describeStep(
        took, agent, anchor, path[pivot], noses[pivot - 1], noses[pivot], pivot - 1)
        + describeStep(
            took, agent, path[pivot], anchor, noses[pivot], noses[pivot + 1], pivot);
    int noseAfterDetour = noses[pivot + 1];
    int tookH = distTable.get(agent, anchor, noseAfterDetour);
    int stayedH = distTable.get(agent, anchor, noses[pivot - 1]);
    StringBuilder stayed = new StringBuilder();
    float stayedTotal = describeStep(
        stayed, agent, anchor, anchor, noses[pivot - 1], noses[pivot - 1], pivot - 1)
        + describeStep(
            stayed, agent, anchor, anchor, noses[pivot - 1], noses[pivot - 1], pivot);
    logger.info(
        "[MAPF guide-detour] construct={} refine={} slot={} a{} leaves {} for {} at t{} and"
            + " returns at t{} | nose before={} detourAfter={} standingAfter={} | detour g={}"
            + " h={} f={} via {} | standing g={} h={} f={} via {}",
        constructions, refineRound, orderSlot,
        agent, roadmap.nameOf(anchor), roadmap.nameOf(path[pivot]), pivot, pivot + 1,
        noseName(noses[pivot - 1]), noseName(noseAfterDetour), noseName(noses[pivot - 1]),
        tookTotal, tookH, tookTotal + tookH, took,
        stayedTotal, stayedH, stayedTotal + stayedH, stayed);
  }

  void traceRolloutCarryRebuilt(
      int construct, int solveDepth, int agent, int at, boolean committedPrefix,
      int[] carried, int[] rebuilt) {
    if (!TRACE_ROLLOUT_CARRY_REBUILT || !logger.isInfoEnabled()) {
      return;
    }
    logger.info(
        "[MAPF guide-rollout] construct={} depth={} a{} at={} prefix={} | carried={} | rebuilt={}"
            + " | next {} -> {}",
        construct, solveDepth, agent, roadmap.nameOf(at), committedPrefix,
        describePath(carried), describePath(rebuilt), nameOf(carried, 1), nameOf(rebuilt, 1));
  }

  private String describePath(int[] path) {
    StringBuilder out = new StringBuilder("[");
    int limit = Math.min(8, path.length);
    for (int i = 0; i < limit; i++) {
      if (i > 0) {
        out.append(", ");
      }
      out.append(path[i] == CollisionTable.NO_VERTEX ? "-" : roadmap.nameOf(path[i]));
    }
    if (path.length > limit) {
      out.append(", …");
    }
    return out.append(']').toString();
  }

  private String nameOf(int[] path, int index) {
    return index >= path.length || path[index] == CollisionTable.NO_VERTEX
        ? "-"
        : roadmap.nameOf(path[index]);
  }

  private void noteCap(int spent, int cap, String what, String property) {
    if (spent != cap) {
      return;
    }
    logger.info(
        "[MAPF guide-backtrack] {} lines capped at {} for this solve, the rest are dropped;"
            + " raise {} to see more",
        what, cap, property);
  }

  private boolean hasImmediateBacktrack(FmsLocalGuide.GuideNode node) {
    for (FmsLocalGuide.GuideNode step = node; step != null && step.parent != null
        && step.parent.parent != null; step = step.parent) {
      if (step.where == step.parent.parent.where && step.where != step.parent.where) {
        return true;
      }
    }
    return false;
  }

  private String describeNode(FmsLocalGuide.GuideNode node) {
    return roadmap.nameOf(node.where) + "@t" + node.when + " nose=" + noseName(node.nose)
        + " g=" + node.g + " h=" + node.h + " f=" + node.f + " moves=" + node.moves;
  }

  private float describeStep(
      StringBuilder out, int agent, int from, int to, int noseFrom, int noseTo, int when) {
    float weight = roadmap.weightOf(to);
    float turn = Roadmap.turnCostBetween(noseFrom, noseTo);
    int collision = collisions.collisionCost(from, to, when);
    List<VertexBlocker> vertexBlockers = vertexBlockersOf(from, to, when);
    float vertexCharge = collision >= 1
        ? scaledCollisionCost + collision * scaledCollisionCostOrder
        : 0.0f;
    int bodies = footprintReservations == null
        ? 0
        : footprintReservations.conflictsAtQuietly(
            new Footprint.Motion(from, to, noseFrom, loaded[agent]), when);
    List<GuideFootprintReservations.Reservation> footprintBlockers = footprintBlockersOf(
        new Footprint.Motion(from, to, noseFrom, loaded[agent]), when);
    float footprintCharge = bodies >= 1
        ? scaledFootprintCost + bodies * scaledCollisionCostOrder
        : 0.0f;
    float unplanned = heldByUnplannedAgent.test(agent, to) ? scaledCollisionCost : 0.0f;
    out.append(out.length() == 0 ? "" : " + ")
        .append(roadmap.nameOf(from)).append("->").append(roadmap.nameOf(to))
        .append("@t").append(when)
        .append("[w=").append(weight)
        .append(" turn=").append(turn)
        .append(" vertex=").append(vertexCharge)
        .append(" footprint=").append(footprintCharge)
        .append(" unplanned=").append(unplanned)
        .append(" vertexBlockers=").append(describeVertexBlockers(vertexBlockers, from, to, when))
        .append(" footprintBlockers=").append(describeFootprintBlockers(footprintBlockers))
        .append(']');
    return weight + turn + vertexCharge + footprintCharge + unplanned;
  }

  private List<VertexBlocker> vertexBlockersOf(int from, int to, int timeFrom) {
    if (from == CollisionTable.NO_VERTEX || to == CollisionTable.NO_VERTEX) {
      return List.of();
    }
    int timeTo = timeFrom + 1;
    List<Integer> arrivingLater = collisions.occupantsAt(to, timeTo);
    List<Integer> arrivingNow = collisions.occupantsAt(to, timeFrom);
    List<VertexBlocker> found = new ArrayList<>();

    for (int agent : arrivingLater) {
      found.add(new VertexBlocker(agent, "vertex"));
    }
    for (int comingBack : collisions.occupantsAt(from, timeTo)) {
      if (arrivingNow.contains(comingBack)) {
        found.add(new VertexBlocker(comingBack, "headOn"));
      }
    }
    if (from != to) {
      for (int occupant : arrivingNow) {
        if (!arrivingLater.contains(occupant)) {
          found.add(new VertexBlocker(occupant, "follower"));
        }
      }
    }
    return List.copyOf(found);
  }

  private String describeVertexBlockers(
      List<VertexBlocker> conflicts, int from, int to, int timeFrom) {
    if (conflicts.isEmpty()) {
      return "[]";
    }
    StringBuilder out = new StringBuilder("[");
    for (VertexBlocker conflict : conflicts) {
      if (out.length() > 1) {
        out.append(',');
      }
      out.append('a').append(conflict.agent())
          .append(':').append(conflict.kind())
          .append('(').append(roadmap.nameOf(from))
          .append("->").append(roadmap.nameOf(to))
          .append("@t").append(timeFrom).append(')');
    }
    return out.append(']').toString();
  }

  private record VertexBlocker(int agent, String kind) {
  }

  private List<GuideFootprintReservations.Reservation> footprintBlockersOf(
      Footprint.Motion candidate, int time) {
    if (footprintReservations == null || footprint == null) {
      return List.of();
    }
    boolean candidateTurns = footprint.isRotation(candidate.nose(), candidate.from(), candidate.to());
    List<GuideFootprintReservations.Reservation> found = new ArrayList<>();
    for (GuideFootprintReservations.Reservation reservation : footprintReservations.reservationsAt(time)) {
      if (!candidateTurns && !reservation.turns()) {
        continue;
      }
      if (footprint.collideMoving(candidate, reservation.motion())) {
        found.add(reservation);
      }
    }
    return List.copyOf(found);
  }

  private String describeFootprintBlockers(
      List<GuideFootprintReservations.Reservation> conflicts) {
    if (conflicts.isEmpty()) {
      return "[]";
    }
    StringBuilder out = new StringBuilder("[");
    for (GuideFootprintReservations.Reservation conflict : conflicts) {
      if (out.length() > 1) {
        out.append(',');
      }
      Footprint.Motion motion = conflict.motion();
      out.append('a').append(conflict.agent())
          .append(':').append(roadmap.nameOf(motion.from()))
          .append("->").append(roadmap.nameOf(motion.to()))
          .append(conflict.turns() ? ":turn" : motion.moves() ? ":move" : ":rest")
          .append(motion.loaded() ? ":loaded" : ":empty");
    }
    return out.append(']').toString();
  }

  boolean wantsRebuildChoice() {
    return TRACE_REBUILD_CHOICE
        && logger.isInfoEnabled() && rebuildChoiceLines < MAX_REBUILD_CHOICE_LINES;
  }

  void traceRebuildChoice(
      int agent, int construct, int refineRound, int orderSlot, int start, int startWhen,
      int startNose, int goal, int[] carried, int[] rebuilt) {
    if (!wantsRebuildChoice()) {
      return;
    }
    int carriedNext = vertexAt(carried, startWhen + 1);
    int rebuiltNext = vertexAt(rebuilt, startWhen + 1);
    if (carriedNext == CollisionTable.NO_VERTEX || rebuiltNext == CollisionTable.NO_VERTEX
        || carriedNext == rebuiltNext || vertexAt(carried, startWhen) != start) {
      return;
    }
    GuidePathCost carriedCost =
        evaluatePathQuietly(agent, carried, startWhen, startNose, goal);
    GuidePathCost rebuiltCost =
        evaluatePathQuietly(agent, rebuilt, startWhen, startNose, goal);
    rebuildChoiceLines++;

    StringBuilder out = new StringBuilder("[MAPF guide-rebuild-choice]\n");
    out.append("  construct=").append(construct)
        .append(" refine=").append(refineRound)
        .append(" slot=").append(orderSlot).append('\n')
        .append("  a").append(agent)
        .append(" start=").append(roadmap.nameOf(start))
        .append(" startWhen=").append(startWhen)
        .append(" goal=").append(roadmap.nameOf(goal))
        .append(" nose=").append(noseName(startNose))
        .append(" loaded=").append(loaded[agent]).append('\n');
    appendScoredPath(out, "carried", carried, carriedCost);
    appendScoredPath(out, "rebuilt", rebuilt, rebuiltCost);
    out.append("  delta rebuilt-carried: f=").append(rebuiltCost.f() - carriedCost.f())
        .append(" g=").append(rebuiltCost.g() - carriedCost.g())
        .append(" h=").append(rebuiltCost.h() - carriedCost.h())
        .append(" weight=").append(rebuiltCost.weight() - carriedCost.weight())
        .append(" turn=").append(rebuiltCost.turn() - carriedCost.turn())
        .append(" vertex=").append(rebuiltCost.vertex() - carriedCost.vertex())
        .append(" footprint=").append(rebuiltCost.footprint() - carriedCost.footprint())
        .append(" unplanned=").append(rebuiltCost.unplanned() - carriedCost.unplanned());
    logger.info("{}", out);
  }

  private void appendScoredPath(
      StringBuilder out, String label, int[] path, GuidePathCost cost) {
    out.append("  ").append(label).append(":\n")
        .append("    ").append(describePath(path)).append('\n')
        .append("    g=").append(cost.g())
        .append(" h=").append(cost.h())
        .append(" f=").append(cost.f()).append('\n')
        .append("    weight=").append(cost.weight())
        .append(" turn=").append(cost.turn())
        .append(" vertex=").append(cost.vertex())
        .append(" footprint=").append(cost.footprint())
        .append(" unplanned=").append(cost.unplanned()).append('\n')
        .append("    steps:\n");
    int shown = Math.min(cost.steps().size(), MAX_REBUILD_CHOICE_STEPS);
    for (int i = 0; i < shown; i++) {
      StepCost step = cost.steps().get(i);
      out.append("      ").append(roadmap.nameOf(step.from()))
          .append("->").append(roadmap.nameOf(step.to()))
          .append("@t").append(step.when()).append('\n')
          .append("        w=").append(step.weight())
          .append(" turn=").append(step.turn())
          .append(" vertex=").append(step.vertex())
          .append(" footprint=").append(step.footprint())
          .append(" unplanned=").append(step.unplanned()).append('\n')
          .append("        vertexBlockers=")
          .append(describeVertexBlockers(
              step.vertexBlockers(), step.from(), step.to(), step.when()))
          .append('\n')
          .append("        footprintBlockers=")
          .append(describeFootprintBlockers(step.footprintBlockers())).append('\n');
    }
    if (cost.steps().size() > shown) {
      out.append("      ... ").append(cost.steps().size() - shown).append(" more steps\n");
    }
  }

  private GuidePathCost evaluatePathQuietly(
      int agent, int[] path, int startWhen, int startNose, int goal) {
    List<StepCost> steps = new ArrayList<>();
    float weight = 0.0f;
    float turn = 0.0f;
    float vertex = 0.0f;
    float footprint = 0.0f;
    float unplanned = 0.0f;
    int nose = startNose;
    int last = vertexAt(path, startWhen);
    int lastNose = startNose;
    for (int t = Math.max(0, startWhen); t + 1 < path.length; t++) {
      int from = path[t];
      int to = path[t + 1];
      if (from == CollisionTable.NO_VERTEX || to == CollisionTable.NO_VERTEX || from == goal) {
        break;
      }
      int noseAfter = roadmap.noseAfter(nose, from, to);
      StepCost step = scoreStepQuietly(agent, from, to, nose, noseAfter, t);
      steps.add(step);
      weight += step.weight();
      turn += step.turn();
      vertex += step.vertex();
      footprint += step.footprint();
      unplanned += step.unplanned();
      nose = noseAfter;
      last = to;
      lastNose = noseAfter;
      if (to == goal) {
        break;
      }
    }
    float g = weight + turn + vertex + footprint + unplanned;
    int h = last == CollisionTable.NO_VERTEX ? 0 : distTable.get(agent, last, lastNose);
    return new GuidePathCost(
        weight, turn, vertex, footprint, unplanned, g, h, g + h, List.copyOf(steps));
  }

  private StepCost scoreStepQuietly(
      int agent, int from, int to, int noseFrom, int noseTo, int when) {
    float weight = roadmap.weightOf(to);
    float turn = Roadmap.turnCostBetween(noseFrom, noseTo);
    int collision = collisions.collisionCost(from, to, when);
    float vertex = collision >= 1
        ? scaledCollisionCost + collision * scaledCollisionCostOrder
        : 0.0f;
    Footprint.Motion motion = new Footprint.Motion(from, to, noseFrom, loaded[agent]);
    int bodies = footprintReservations == null
        ? 0
        : footprintReservations.conflictsAtQuietly(motion, when);
    float footprint = bodies >= 1
        ? scaledFootprintCost + bodies * scaledCollisionCostOrder
        : 0.0f;
    float unplanned = heldByUnplannedAgent.test(agent, to) ? scaledCollisionCost : 0.0f;
    return new StepCost(
        from, to, when, weight, turn, vertex, footprint, unplanned,
        vertexBlockersOf(from, to, when), footprintBlockersOf(motion, when));
  }

  private static int vertexAt(int[] path, int index) {
    return index < 0 || index >= path.length ? CollisionTable.NO_VERTEX : path[index];
  }

  private record StepCost(
      int from, int to, int when, float weight, float turn, float vertex, float footprint,
      float unplanned, List<VertexBlocker> vertexBlockers,
      List<GuideFootprintReservations.Reservation> footprintBlockers) {
  }

  private record GuidePathCost(
      float weight, float turn, float vertex, float footprint, float unplanned,
      float g, int h, float f, List<StepCost> steps) {
  }

  private static String noseName(int nose) {
    return switch (nose) {
      case Roadmap.DIR_POS_X -> "+X";
      case Roadmap.DIR_NEG_X -> "-X";
      case Roadmap.DIR_POS_Y -> "+Y";
      case Roadmap.DIR_NEG_Y -> "-Y";
      default -> "NONE";
    };
  }
}
