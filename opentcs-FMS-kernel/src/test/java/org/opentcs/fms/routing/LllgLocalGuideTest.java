package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class LllgLocalGuideTest {

  private static final List<String> PROPERTIES = List.of(
      "fms.guide.on",
      "fms.lllg.window",
      "fms.lllg.numRefine",
      "fms.lllg.deterministic",
      "fms.lllg.collisionCost",
      "fms.lllg.clearGoalFirst");

  @AfterEach
  void restoreDefaults() {
    PROPERTIES.forEach(System::clearProperty);
  }

  @Test
  void plansOnlyInVertexTimeAndIgnoresTheVehicleNose() {
    Roadmap lane = GridFixture.grid(1, 3);
    int current = GridFixture.id(lane, 0, 0);
    int goal = GridFixture.id(lane, 2, 0);
    int[] configuration = {current};
    int[] order = {0};
    configureDeterministicGuide();

    LllgLocalGuide facingEast = newGuide(lane, goal);
    facingEast.construct(configuration, new int[] {Roadmap.DIR_POS_X}, order, 0);
    LllgLocalGuide facingNorth = newGuide(lane, goal);
    facingNorth.construct(configuration, new int[] {Roadmap.DIR_POS_Y}, order, 0);

    assertThat(facingEast.currentGuidePaths()[0])
        .containsExactly(current, GridFixture.id(lane, 1, 0), goal, goal, goal);
    assertThat(facingNorth.currentGuidePaths())
        .isEqualTo(facingEast.currentGuidePaths());
    assertThat(facingEast.footprintConflictsOf(0)).isZero();
    assertThat(facingEast.collisionCostOf(0)).isZero();
  }

  @Test
  void restoresTheGuidedStepWithTheParentPaths() {
    Roadmap lane = GridFixture.grid(1, 3);
    int current = GridFixture.id(lane, 0, 0);
    int next = GridFixture.id(lane, 1, 0);
    int goal = GridFixture.id(lane, 2, 0);
    configureDeterministicGuide();

    LllgLocalGuide guide = newGuide(lane, goal);
    guide.setGuidePaths(new int[][] {{current, next, goal}});

    assertThat(guide.guidedConfiguration()).containsExactly(next);
  }

  private static LllgLocalGuide newGuide(Roadmap roadmap, int goal) {
    return new LllgLocalGuide(
        roadmap, DistTable.weighted(roadmap, new int[] {goal}), new int[] {goal}, 1, 0);
  }

  private static void configureDeterministicGuide() {
    System.setProperty("fms.guide.on", "true");
    System.setProperty("fms.lllg.window", "5");
    System.setProperty("fms.lllg.numRefine", "1");
    System.setProperty("fms.lllg.deterministic", "true");
  }
}
