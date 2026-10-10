package org.opentcs.fms.routing;

import java.util.ArrayList;
import java.util.List;
import org.opentcs.fms.routing.diag.GuideCounters;
import org.opentcs.fms.routing.pibt.Footprint;

/** Time-indexed motions enrolled by LocalGuide for turn-footprint checks. */
final class GuideFootprintReservations {

  private final Footprint footprint;
  private final List<List<Reservation>> timeline;

  private final GuideCounters counters;

  GuideFootprintReservations(Footprint footprint, int window, GuideCounters counters) {
    this.footprint = footprint;
    this.counters = counters;
    this.timeline = new ArrayList<>(Math.max(0, window - 1));
    for (int time = 0; time + 1 < window; time++) {
      timeline.add(new ArrayList<>());
    }
  }

  void enroll(int agent, int[] path, int startNose, boolean loaded) {
    int nose = startNose;
    for (int time = 0; time < timeline.size(); time++) {
      int from = path[time];
      int to = path[time + 1];
      if (from == CollisionTable.NO_VERTEX || to == CollisionTable.NO_VERTEX) {
        break;
      }
      Footprint.Motion motion = new Footprint.Motion(from, to, nose, loaded);
      timeline.get(time).add(new Reservation(agent, motion, turns(motion)));
      nose = footprint.noseAfter(nose, from, to);
    }
  }

  void enrollRestingBody(int agent, int at, int nose, boolean loaded, int ticks) {
    Footprint.Motion resting = Footprint.Motion.resting(at, nose, loaded);
    for (int time = 0; time < Math.min(ticks, timeline.size()); time++) {
      timeline.get(time).add(new Reservation(agent, resting, false));
    }
  }

  void clear(int agent) {
    for (List<Reservation> atTime : timeline) {
      atTime.removeIf(reservation -> reservation.agent() == agent);
    }
  }

  int conflictsAt(Footprint.Motion candidate, int time) {
    return count(candidate, time, true);
  }

  int conflictsAtQuietly(Footprint.Motion candidate, int time) {
    return count(candidate, time, false);
  }

  private int count(Footprint.Motion candidate, int time, boolean tally) {
    if (time < 0 || time >= timeline.size()) {
      return 0;
    }
    boolean candidateTurns = turns(candidate);
    int found = 0;
    for (Reservation reservation : timeline.get(time)) {
      if (!candidateTurns && !reservation.turns()) {
        continue;
      }
      if (tally) {
        counters.footprintChecks++;
      }
      if (footprint.collideMoving(candidate, reservation.motion())) {
        found++;
        if (!tally) {
          continue;
        }
        counters.footprintConflicts++;
        if (candidateTurns && reservation.turns()) {
          counters.bothTurn++;
        } else if (candidateTurns) {
          if (reservation.motion().moves()) {
            counters.myTurnAgainstABodyThatIsPassingThrough++;
          } else {
            counters.myTurnAgainstABodyThatIsStandingStill++;
          }
        } else {
          counters.theirTurnSweepsMeWhileIStandStill++;
        }
      }
    }
    return found;
  }

  List<Reservation> reservationsAt(int time) {
    if (time < 0 || time >= timeline.size()) {
      return List.of();
    }
    return List.copyOf(timeline.get(time));
  }

  private boolean turns(Footprint.Motion motion) {
    return footprint.isRotation(motion.nose(), motion.from(), motion.to());
  }

  record Reservation(int agent, Footprint.Motion motion, boolean turns) {
  }
}
