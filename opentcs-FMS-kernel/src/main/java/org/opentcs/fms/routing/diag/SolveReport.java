package org.opentcs.fms.routing.diag;

import org.opentcs.fms.routing.Roadmap;

/** Renders solver tallies into the log lines; holds every format string in one place. */
public final class SolveReport {

  private static final int HEAD_CELLS = 6;

  private SolveReport() {
  }

  public static String pibtChoices(PibtCounters c, boolean hindranceEnabled) {
    StringBuilder out = new StringBuilder();
    out.append("hindrance=").append(hindranceEnabled)
        .append(" hindranceDecisions=").append(c.hindranceDecisions)
        .append(" guidanceDecisions=").append(c.guidanceDecisions)
        .append(" guidedRefused=").append(c.guidedVertexRefused)
        .append(" exitPreferenceDecisions=").append(c.exitPreferenceDecisions)
        .append(" awayFromGoal=").append(c.stepsAwayFromGoal)
        .append(" choseToStay=").append(c.choseToStay)
        .append(" heldInPlace=").append(c.stepsHeldInPlace)
        .append(" evictedFromOwnCell=").append(c.evictedFromOwnCell)
        .append(" guideBlindToTheClash=").append(c.guidedCellRefusedForAClashTheGuideNeverSaw)
        .append(" guidePaidYetRefused=").append(c.guidedCellRefusedForAClashTheGuidePaidFor)
        .append(" | firstChoiceBlockedBy:");
    appendTally(out, PibtCounters.SKIP_REASONS, c.blockedFirstChoice);
    out.append(" | skippedTotal:");
    appendTally(out, PibtCounters.SKIP_REASONS, c.skipped);
    return out.toString();
  }

  public static String pibtFailures(PibtCounters c) {
    StringBuilder out = new StringBuilder();
    appendTally(out, PibtCounters.FAILURE_CAUSES, c.failures);
    if (c.firstFailureCause >= 0) {
      out.append(" | first=").append(PibtCounters.FAILURE_CAUSES[c.firstFailureCause])
          .append(' ').append(c.firstFailureDetail);
    }
    return out.toString();
  }

  public static String guideActivity(
      GuideCounters c, int agents, int window, int guideConstructions,
      int guidanceDecisions, int hindranceDecisions) {
    return new StringBuilder()
        .append("agents=").append(agents)
        .append(" window=").append(window)
        .append(" constructs=").append(guideConstructions)
        .append(" warmStartSlides=").append(c.warmStartSlides)
        .append(" carryForwardSlides=").append(c.carryForwardSlides)
        .append(" guidanceDecisions=").append(guidanceDecisions)
        .append(" hindranceDecisions=").append(hindranceDecisions)
        .append(" footprintChecks=").append(c.footprintChecks)
        .append(" footprintConflicts=").append(c.footprintConflicts)
        .append(" | carriedPath: followed=").append(c.warmFollowedItsCarriedPath)
        .append(" anchoredNoTick=").append(c.warmAnchoredWithoutSpendingATick)
        .append(" standsOff=").append(c.warmStandsOffItsCarriedPath)
        .append(" noSeed=").append(c.warmHasNoCarriedPathAtAll)
        .append(" | atGoalReleased=").append(c.agentsReleasedBecauseTheyAreAtTheirGoal)
        .append(" bodiesInTheWay(1/2/3/4+)=").append(c.bodiesInTheWayWhenCharged[1])
        .append('/').append(c.bodiesInTheWayWhenCharged[2])
        .append('/').append(c.bodiesInTheWayWhenCharged[3])
        .append('/').append(c.bodiesInTheWayWhenCharged[4])
        .append(" | charged(of ").append(c.nodesExpanded).append(" nodes):")
        .append(" footprint=").append(c.chargedForABodyInTheWay)
        .append(" vertex=").append(c.chargedForACellAnotherPlanAlreadyUses)
        .append(" unplanned=").append(c.chargedForACellHeldByAnAgentThatHasNotPlannedYet)
        .append(" turn=").append(c.chargedForTurning)
        .append(" | clashKind: myTurnVsStanding=").append(c.myTurnAgainstABodyThatIsStandingStill)
        .append(" myTurnVsPassing=").append(c.myTurnAgainstABodyThatIsPassingThrough)
        .append(" theirTurnVsMe=").append(c.theirTurnSweepsMeWhileIStandStill)
        .append(" bothTurn=").append(c.bothTurn)
        .toString();
  }

  public static String carriedPathHead(Roadmap roadmap, int[] carriedPath, int noVertex) {
    StringBuilder head = new StringBuilder();
    for (int t = 0; t < Math.min(HEAD_CELLS, carriedPath.length); t++) {
      head.append(t == 0 ? "" : ", ")
          .append(carriedPath[t] == noVertex ? "-" : roadmap.nameOf(carriedPath[t]));
    }
    return head.toString();
  }

  public static String whereTheCarriedPathHolds(int[] carriedPath, int cell) {
    for (int t = 0; t < carriedPath.length; t++) {
      if (carriedPath[t] == cell) {
        return "at index " + t;
      }
    }
    return "nowhere";
  }

  private static void appendTally(StringBuilder out, String[] labels, int[] values) {
    for (int i = 0; i < labels.length; i++) {
      out.append(' ').append(labels[i]).append('=').append(values[i]);
    }
  }
}
