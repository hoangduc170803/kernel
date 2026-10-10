package org.opentcs.fms.routing.diag;

/** Mutable tallies written by PibtEngine while it decides a configuration. */
public final class PibtCounters {

  public static final String[] SKIP_REASONS = {
      "cellTaken", "footprintVsPlanned", "footprintVsUnplanned", "follower"
  };
  public static final String[] FAILURE_CAUSES = {
      "constraintCellTaken", "constraintUnreachable", "agentOutOfOptions", "bodiesOverlap",
      "constraintFollower"
  };

  public int firstFailureCause = -1;
  public String firstFailureDetail = "";

  public int turnsConsidered;
  public int footprintRejections;
  public int footprintPushes;
  public int overlappingConfigurations;
  public int hindranceDecisions;
  public int guidanceDecisions;
  public int exitPreferenceDecisions;
  public int guidedVertexRefused;
  public int guidedCellRefusedForAClashTheGuideNeverSaw;
  public int guidedCellRefusedForAClashTheGuidePaidFor;
  public int stepsAwayFromGoal;
  public int stepsHeldInPlace;
  public int evictedFromOwnCell;
  public int choseToStay;
  public int agentsLeftOpen;
  public int replans;
  public int abandonedPushes;

  public int[] skipped;
  public int[] blockedFirstChoice;
  public int[] failures;

  public PibtCounters() {
    this.skipped = new int[SKIP_REASONS.length];
    this.blockedFirstChoice = new int[SKIP_REASONS.length];
    this.failures = new int[FAILURE_CAUSES.length];
  }
}
