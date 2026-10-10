package org.opentcs.fms.kernel.mapf;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.AgentPriority;
import org.opentcs.fms.routing.Lacam2Solver;
import org.opentcs.fms.routing.LegalMoves;
import org.opentcs.fms.routing.MapfInstance;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.SolverParams;
import org.opentcs.fms.routing.StaticBodies;

class MapfSolveCaptureTest {

  @Test
  void serializedCaptureReplaysTheSameRawPaths() {
    Roadmap roadmap = new Roadmap(new Roadmap.Model(
        List.of(
            new Roadmap.Point("A", 0, 0, 0),
            new Roadmap.Point("B", 1000, 0, 0)),
        List.of(new Roadmap.Path("A-B", "A", "B", 1000, true))));
    MapfInstance instance = new MapfInstance(
        new int[] { roadmap.idOf("A") }, new int[] { roadmap.idOf("B") });
    SolverParams params = new SolverParams(3, 30_000, 42);
    AgentPriority[] priorities = { AgentPriority.NEWCOMER };
    int[] startAxis = { Roadmap.DIR_NONE };
    int[] startMotion = { Roadmap.DIR_NONE };
    boolean[] loaded = { false };
    LegalMoves moves = new LegalMoves(roadmap, null);
    Lacam2Solver.Result result = new Lacam2Solver(roadmap).solve(
        instance, params.deadlineMs(), params.seed(), Lacam2Solver.Objective.NONE,
        Lacam2Solver.DEFAULT_RESTART_RATE, 100, params.window(), priorities, startAxis,
        loaded, null, null, StaticBodies.NONE, startMotion, moves);
    MapfSolveCapture capture = MapfSolveCapture.capture(
        "capture-test", roadmap, Set.of(), List.of("Vehicle-1"), instance, params,
        Lacam2Solver.Objective.NONE, Lacam2Solver.DEFAULT_RESTART_RATE, 100,
        priorities, startAxis, loaded, null, null, List.of(), StaticBodies.NONE,
        startMotion, moves, result);

    MapfSolveCapture restored = MapfSolveCapture.fromJson(capture.toJson());

    assertTrue(restored.result().solved());
    assertEquals(capture.result().rawPaths(), restored.replayRawPaths(roadmap));
  }
}
