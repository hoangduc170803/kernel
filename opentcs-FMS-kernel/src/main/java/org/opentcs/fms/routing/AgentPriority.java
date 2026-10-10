package org.opentcs.fms.routing;

public record AgentPriority(
    int retreating, int inGoalLane, int aged, int settled, float tie, int previousOrderRank)
    implements Comparable<AgentPriority> {

  public static final AgentPriority NEWCOMER = new AgentPriority(
      0, 0, 0, 0, 0f, Integer.MAX_VALUE);

  public AgentPriority(int retreating, int inGoalLane, int aged, int settled, float tie) {
    this(retreating, inGoalLane, aged, settled, tie, Integer.MAX_VALUE);
  }

  @Override
  public int compareTo(AgentPriority other) {
    if (retreating != other.retreating) {
      return Integer.compare(retreating, other.retreating);
    }
    if (inGoalLane != other.inGoalLane) {
      return Integer.compare(inGoalLane, other.inGoalLane);
    }
    if (aged != other.aged) {
      return Integer.compare(aged, other.aged);
    }
    if (settled != other.settled) {
      return Integer.compare(settled, other.settled);
    }
    if (Float.compare(tie, other.tie) != 0) {
      return Float.compare(tie, other.tie);
    }
    return Integer.compare(other.previousOrderRank, previousOrderRank);
  }
}
