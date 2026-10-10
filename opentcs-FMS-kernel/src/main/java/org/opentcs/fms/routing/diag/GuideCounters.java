package org.opentcs.fms.routing.diag;

/** Mutable tallies written by LocalGuide and its footprint reservations. */
public final class GuideCounters {

  public int constructions;
  public int carryForwardSlides;
  public int warmStartSlides;
  public int agentsReleasedBecauseTheyAreAtTheirGoal;
  public final int[] bodiesInTheWayWhenCharged = new int[5];

  public int chargedForABodyInTheWay;
  public int chargedForACellAnotherPlanAlreadyUses;
  public int chargedForACellHeldByAnAgentThatHasNotPlannedYet;
  public int chargedForTurning;
  public int nodesExpanded;

  public int followedItsCarriedPath;
  public int anchoredWithoutSpendingATick;
  public int standsOffItsCarriedPath;
  public int hasNoCarriedPathAtAll;

  public int warmFollowedItsCarriedPath;
  public int warmAnchoredWithoutSpendingATick;
  public int warmStandsOffItsCarriedPath;
  public int warmHasNoCarriedPathAtAll;

  public int footprintChecks;
  public int footprintConflicts;
  public int myTurnAgainstABodyThatIsStandingStill;
  public int myTurnAgainstABodyThatIsPassingThrough;
  public int theirTurnSweepsMeWhileIStandStill;
  public int bothTurn;
}
