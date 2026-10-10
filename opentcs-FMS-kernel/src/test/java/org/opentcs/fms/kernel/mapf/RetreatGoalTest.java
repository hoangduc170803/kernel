package org.opentcs.fms.kernel.mapf;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.Roadmap;

class RetreatGoalTest {

  private static Roadmap laneWithOneWayExit() {
    List<Roadmap.Point> points = List.of(
        new Roadmap.Point("c122", 0, 0, 0),
        new Roadmap.Point("c132", 0, 1, 0),
        new Roadmap.Point("c142", 0, 2, 0),
        new Roadmap.Point("c152", 0, 3, 0),
        new Roadmap.Point("c162", 0, 4, 0),
        new Roadmap.Point("c121", 1, 0, 0),
        new Roadmap.Point("c131", 1, 1, 0),
        new Roadmap.Point("c141", 1, 2, 0));
    List<Roadmap.Path> paths = List.of(
        new Roadmap.Path("c122-c132", "c122", "c132", 1, true),
        new Roadmap.Path("c132-c142", "c132", "c142", 1, true),
        new Roadmap.Path("c142-c152", "c142", "c152", 1, true),
        new Roadmap.Path("c152-c162", "c152", "c162", 1, true),
        new Roadmap.Path("c142-c141", "c142", "c141", 1, false),
        new Roadmap.Path("c141-c131", "c141", "c131", 1, false),
        new Roadmap.Path("c131-c121", "c131", "c121", 1, false),
        new Roadmap.Path("c121-c122", "c121", "c122", 1, false));
    return new Roadmap(new Roadmap.Model(
        points, paths, Set.of(), Set.of("c142", "c152", "c162"), false));
  }

  private static int[] path(Roadmap map, String... cells) {
    int[] vertices = new int[cells.length];
    for (int time = 0; time < cells.length; time++) {
      vertices[time] = map.idOf(cells[time]);
    }
    return vertices;
  }

  private static int[] holding(Roadmap map, String cell, int ticks) {
    int[] vertices = new int[ticks];
    java.util.Arrays.fill(vertices, map.idOf(cell));
    return vertices;
  }

  @Test
  void aRetreatingVehicleStopsOnTheCellWesToldItToWaitOn() {
    Roadmap map = laneWithOneWayExit();
    int[] retreatPath = path(map, "c162", "c152", "c142", "c142", "c141", "c131", "c121");
    List<AgentWindowPlan> plans = List.of(
        new AgentWindowPlan(retreatPath, map.idOf("c142"), true),
        new AgentWindowPlan(holding(map, "c122", 7), map.idOf("c122"), false));

    int[][] adjustedPaths = RetreatGoal.adjust(plans);

    assertThat(adjustedPaths[0])
        .as("everything after the first touch of c142 becomes standing still on c142")
        .containsExactly(path(map, "c162", "c152", "c142", "c142", "c142", "c142", "c142"));
    assertThat(plans.get(0).path())
        .as("applying the policy does not mutate the solver output")
        .containsExactly(path(map, "c162", "c152", "c142", "c142", "c141", "c131", "c121"));
  }

  @Test
  void aVehicleThatIsNotOnARetreatLegKeepsItsPlanUntouched() {
    Roadmap map = laneWithOneWayExit();
    int[] before = path(map, "c162", "c152", "c142", "c141", "c131");
    List<AgentWindowPlan> plans = List.of(
        new AgentWindowPlan(before, map.idOf("c142"), false));

    int[][] adjustedPaths = RetreatGoal.adjust(plans);

    assertThat(adjustedPaths[0])
        .as("ordinary MAPF semantics: a goal is just a coordinate and driving through is allowed")
        .containsExactly(before);
  }

  @Test
  void aRetreatingVehicleIsHeldAtItsGoalAfterTheFirstTouch() {
    Roadmap map = laneWithOneWayExit();
    int[] retreatPath = path(map, "c162", "c152", "c142", "c141", "c131", "c121");
    List<AgentWindowPlan> plans = List.of(
        new AgentWindowPlan(retreatPath, map.idOf("c142"), true),
        new AgentWindowPlan(
            path(map, "c122", "c122", "c122", "c132", "c142", "c152"),
            map.idOf("c152"), false));

    int[][] adjustedPaths = RetreatGoal.adjust(plans);

    assertThat(adjustedPaths[0])
        .as("the retreat priority makes c142 terminal for V8")
        .containsExactly(path(map, "c162", "c152", "c142", "c142", "c142", "c142"));
  }

  @Test
  void aPlanThatAlreadyStopsOnTheGoalIsLeftAlone() {
    Roadmap map = laneWithOneWayExit();
    int[] before = path(map, "c162", "c152", "c142", "c142", "c142");
    List<AgentWindowPlan> plans = List.of(
        new AgentWindowPlan(before, map.idOf("c142"), true));

    int[][] adjustedPaths = RetreatGoal.adjust(plans);

    assertThat(adjustedPaths[0]).containsExactly(before);
  }

  @Test
  void aPlanThatNeverReachesTheGoalInsideTheWindowIsLeftAlone() {
    Roadmap map = laneWithOneWayExit();
    int[] before = path(map, "c162", "c152", "c152", "c152");
    List<AgentWindowPlan> plans = List.of(
        new AgentWindowPlan(before, map.idOf("c142"), true));

    int[][] adjustedPaths = RetreatGoal.adjust(plans);

    assertThat(adjustedPaths[0])
        .as("nothing to adjust: the vehicle has not arrived yet, the window just ran out")
        .containsExactly(before);
  }
}
