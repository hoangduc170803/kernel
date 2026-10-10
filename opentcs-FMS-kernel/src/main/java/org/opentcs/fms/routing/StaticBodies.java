package org.opentcs.fms.routing;

import java.util.List;
import org.opentcs.fms.routing.pibt.Footprint;

public record StaticBodies(
    int[] cells, int[] noses, boolean[] loaded, List<Footprint.Obstacle> obstacles) {

  public static final StaticBodies NONE
      = new StaticBodies(new int[0], new int[0], new boolean[0]);

  public StaticBodies(int[] cells, int[] noses, boolean[] loaded) {
    this(cells, noses, loaded, List.of());
  }

  public StaticBodies {
    obstacles = List.copyOf(obstacles);
  }

  public int size() {
    return cells.length;
  }

  public StaticBodies withObstacles(List<Footprint.Obstacle> extra) {
    return new StaticBodies(cells, noses, loaded, extra);
  }

  public boolean hitBy(Footprint footprint, Footprint.Motion motion) {
    return hitsAVehicleBody(footprint, motion) || hitsAnObstacle(footprint, motion);
  }

  private boolean hitsAVehicleBody(Footprint footprint, Footprint.Motion motion) {
    for (int body = 0; body < size(); body++) {
      int cell = cells[body];
      if (!footprint.mayInteract(motion.from(), cell)) {
        continue;
      }
      if (footprint.collideMoving(
          motion,
          Footprint.Motion.resting(cell, noses[body], loaded[body]))) {
        return true;
      }
    }
    return false;
  }

  private boolean hitsAnObstacle(Footprint footprint, Footprint.Motion motion) {
    for (Footprint.Obstacle obstacle : obstacles) {
      if (footprint.mayInteract(motion.from(), obstacle)
          && footprint.collideMoving(motion, obstacle)) {
        return true;
      }
    }
    return false;
  }
}
