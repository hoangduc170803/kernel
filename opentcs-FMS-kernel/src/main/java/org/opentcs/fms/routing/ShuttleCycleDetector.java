package org.opentcs.fms.routing;

import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.Collections;
import org.opentcs.fms.routing.pibt.Footprint;

public final class ShuttleCycleDetector {

  public record State(int depth, int[] config, int[] axis) {
  }

  public record Match(int ancestorDepth, Set<String> neighborhood) {
  }

  private ShuttleCycleDetector() {
  }

  public static Match detect(
      Roadmap roadmap, Footprint footprint, int[] goals, int agent, State current,
      State parent, List<State> ancestors, StaticBodies statics, int maximumTicks) {
    if (current.config()[agent] == goals[agent]
        || current.config()[agent] == parent.config()[agent]) {
      return null;
    }
    for (State ancestor : ancestors) {
      int distance = current.depth() - ancestor.depth();
      if (distance < 2 || distance > maximumTicks
          || current.config()[agent] != ancestor.config()[agent]
          || current.axis()[agent] != ancestor.axis()[agent]) {
        continue;
      }
      Set<String> currentNeighborhood = neighborhood(
          roadmap, footprint, current.config(), current.axis(), agent, statics);
      Set<String> ancestorNeighborhood = neighborhood(
          roadmap, footprint, ancestor.config(), ancestor.axis(), agent, statics);
      if (currentNeighborhood.equals(ancestorNeighborhood)) {
        return new Match(ancestor.depth(), currentNeighborhood);
      }
    }
    return null;
  }

  private static Set<String> neighborhood(
      Roadmap roadmap, Footprint footprint, int[] config, int[] axis, int agent,
      StaticBodies statics) {
    int cell = config[agent];
    Set<String> result = new TreeSet<>();
    for (int other = 0; other < config.length; other++) {
      if (other == agent || !withinRange(roadmap, footprint, cell, config[other])) {
        continue;
      }
      result.add("a" + other + "@" + roadmap.nameOf(config[other])
          + "/" + axisName(axis[other]));
    }
    for (int body = 0; body < statics.size(); body++) {
      int bodyCell = statics.cells()[body];
      if (withinRange(roadmap, footprint, cell, bodyCell)) {
        result.add("staticBody#" + body + "@" + roadmap.nameOf(bodyCell)
            + "/nose=" + statics.noses()[body] + "/loaded=" + statics.loaded()[body]);
      }
    }
    return Collections.unmodifiableSet(result);
  }

  private static boolean withinRange(Roadmap roadmap, Footprint footprint, int first, int second) {
    if (footprint != null) {
      return footprint.mayInteract(first, second);
    }
    for (int neighbor : roadmap.neighborsOf(first)) {
      if (neighbor == second) {
        return true;
      }
    }
    return false;
  }

  private static String axisName(int axis) {
    return switch (axis) {
      case Roadmap.AXIS_X -> "X";
      case Roadmap.AXIS_Y -> "Y";
      default -> "none";
    };
  }
}
