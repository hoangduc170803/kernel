package org.opentcs.fms.routing;

public final class DistTable {

  public enum Mode {
    WEIGHTED,
    HOPS,
    TURN_AWARE,
    WORK
  }

  private final Roadmap roadmap;
  private final Mode mode;
  private final int unreachable;
  private final int[][] byAgent;

  public static DistTable weighted(Roadmap roadmap, int[] goals) {
    return new DistTable(roadmap, goals, Mode.WEIGHTED);
  }

  public static DistTable hops(Roadmap roadmap, int[] goals) {
    return new DistTable(roadmap, goals, Mode.HOPS);
  }

  public static DistTable turnAware(Roadmap roadmap, int[] goals) {
    return new DistTable(roadmap, goals, Mode.TURN_AWARE);
  }

  public static DistTable work(Roadmap roadmap, int[] goals) {
    return new DistTable(roadmap, goals, Mode.WORK);
  }

  private DistTable(Roadmap roadmap, int[] goals, Mode mode) {
    this.roadmap = roadmap;
    this.mode = mode;
    this.unreachable = switch (mode) {
      case HOPS -> roadmap.size();
      case WEIGHTED -> costThatOutrunsAnyRoute(roadmap);
      case TURN_AWARE -> costThatOutrunsAnyTurningRoute(roadmap);
      case WORK -> costThatOutrunsAnyWorkRoute(roadmap);
    };
    this.byAgent = new int[goals.length][];
    for (int agent = 0; agent < goals.length; agent++) {
      byAgent[agent] = boundedDistancesTo(goals[agent]);
    }
  }

  public int agents() {
    return byAgent.length;
  }

  public int unreachable() {
    return unreachable;
  }

  public boolean turnAware() {
    return mode == Mode.TURN_AWARE || mode == Mode.WORK;
  }

  public int get(int agent, int vertex) {
    return get(agent, vertex, Roadmap.DIR_NONE);
  }

  public int get(int agent, int vertex, int nose) {
    return turnAware()
        ? Roadmap.distanceAt(byAgent[agent], vertex, nose)
        : byAgent[agent][vertex];
  }

  public int[] tableFor(int agent) {
    return byAgent[agent];
  }

  public void update(int agent, int newGoal) {
    byAgent[agent] = boundedDistancesTo(newGoal);
  }

  private int[] boundedDistancesTo(int goal) {
    int[] source = switch (mode) {
      case HOPS -> roadmap.hopDistanceTo(goal);
      case WEIGHTED -> roadmap.distanceTo(goal);
      case TURN_AWARE -> roadmap.turnAwareDistanceTo(goal);
      case WORK -> roadmap.workDistanceTo(goal);
    };
    int[] bounded = new int[source.length];
    for (int state = 0; state < source.length; state++) {
      bounded[state] = Math.min(source[state], unreachable);
    }
    return bounded;
  }

  private static int costThatOutrunsAnyTurningRoute(Roadmap roadmap) {
    return (int) Math.min(
        Integer.MAX_VALUE, (long) roadmap.size() * (roadmap.maxVertexWeight() + 2) + 1);
  }

  private static int costThatOutrunsAnyWorkRoute(Roadmap roadmap) {
    return (int) Math.min(
        Integer.MAX_VALUE, (long) roadmap.size() * (Roadmap.MAX_STEP_WORK) + 1);
  }

  private static int costThatOutrunsAnyRoute(Roadmap roadmap) {
    return (int) Math.min(
        Integer.MAX_VALUE, (long) roadmap.size() * roadmap.maxVertexWeight() + 1);
  }
}
