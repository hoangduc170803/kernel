package org.opentcs.fms.kernel.mapf;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.Roadmap;

class GoalPathCompletionTest {

  @Test
  void completesPathsToTheirGoalsAndAlignsTheirTimelines() {
    Roadmap map = RoadmapFixtures.columnRoadmap("A", "B", "C");
    int goal = map.idOf("C");
    int[][] paths = {
        {map.idOf("A")},
        {map.idOf("B")}
    };
    int[][] distances = {map.distanceTo(goal), map.distanceTo(map.idOf("B"))};

    GoalPathCompletion.Result result = GoalPathCompletion.complete(
        map, paths, new int[] {goal, map.idOf("B")}, distances);

    assertThat(result.paths()[0]).containsExactly(
        map.idOf("A"), map.idOf("B"), map.idOf("C"));
    assertThat(result.paths()[1]).containsExactly(
        map.idOf("B"), map.idOf("B"), map.idOf("B"));
    assertThat(result.strandedAgents()).isEmpty();
  }

  @Test
  void reportsPathsThatCannotFollowTheDistanceGradient() {
    Roadmap map = RoadmapFixtures.columnRoadmap("A", "B", "C");
    int goal = map.idOf("C");
    int[] path = {map.idOf("A")};

    GoalPathCompletion.Result result = GoalPathCompletion.complete(
        map, new int[][] {path}, new int[] {goal}, new int[][] {new int[map.size()]});

    assertThat(result.paths()[0]).containsExactly(path);
    assertThat(result.strandedAgents()).containsExactly(0);
    assertThat(path).containsExactly(map.idOf("A"));
  }

  @Test
  void anEmptyPathStartsAtItsGoal() {
    Roadmap map = RoadmapFixtures.columnRoadmap("A", "B");
    int goal = map.idOf("B");

    GoalPathCompletion.Result result = GoalPathCompletion.complete(
        map, new int[][] {{}}, new int[] {goal}, new int[][] {map.distanceTo(goal)});

    assertThat(result.paths()[0]).containsExactly(goal);
    assertThat(result.strandedAgents()).isEmpty();
  }
}
