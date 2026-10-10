package org.opentcs.fms.kernel.mapf;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;
import static org.opentcs.fms.kernel.mapf.RoadmapFixtures.startsAt;

import java.io.File;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.RoadmapXmlLoader;

class GoalClaimerTest {

  private static final String MAP_FILE = "v7-vda5050-test.xml";

  private static Roadmap map;

  @BeforeAll
  static void loadTheMapTheFleetActuallyDrivesOn() throws Exception {
    File file = RoadmapXmlLoader.findMap(MAP_FILE);
    assumeTrue(file != null, MAP_FILE + " not found; skipping");
    map = new Roadmap(RoadmapXmlLoader.load(file));
  }

  private static List<String> namesOf(int[] vertices) {
    return Arrays.stream(vertices).mapToObj(map::nameOf).toList();
  }

  @Test
  void everyAgentGetsExactlyTheGoalWesAskedFor() {
    GoalClaims claims = new GoalClaimer(map).claim(
        map, List.of("V1", "V2"), Map.of("V1", "3010", "V2", "3011"),
        startsAt(map, "0005", "0006"));

    assertThat(namesOf(claims.goalVertices())).containsExactly("3010", "3011");
    assertThat(claims.stranded()).isEmpty();
  }

  @Test
  void twoAgentsOnTheSameGoalBothKeepIt() {
    GoalClaims claims = new GoalClaimer(map).claim(
        map, List.of("V1", "V2"), Map.of("V1", "3010", "V2", "3010"),
        startsAt(map, "0005", "0006"));

    assertThat(namesOf(claims.goalVertices()))
        .as("goals come from WES, which hands out a distinct target per vehicle; the planner no"
            + " longer substitutes one, so a clash here is a WES bug to surface, not to paper over")
        .containsExactly("3010", "3010");
    assertThat(claims.stranded()).isEmpty();
  }

  @Test
  void anAgentWithoutAGoalStaysWhereItIs() {
    int[] starts = startsAt(map, "0005");

    GoalClaims claims =
        new GoalClaimer(map).claim(map, List.of("V1"), Map.of(), starts);

    assertThat(claims.goalVertices()[0]).isEqualTo(starts[0]);
    assertThat(claims.stranded()).isEmpty();
  }

  @Test
  void anAgentWhoseGoalIsWalledOffIsHeldAndReported() {
    int[] starts = startsAt(map, "0005");
    int goal = map.idOf("3010");
    Roadmap walled = map.blocking(Set.copyOf(neighboursOf(starts[0])));

    GoalClaims claims =
        new GoalClaimer(map).claim(walled, List.of("V1"), Map.of("V1", "3010"), starts);

    assertThat(claims.goalVertices()[0])
        .as("nothing is substituted; the vehicle is pinned where it stands and the walled-off"
            + " goal is reported, so the cause stays visible instead of turning into a detour")
        .isEqualTo(starts[0]);
    assertThat(claims.stranded()).hasSize(1);
    assertThat(map.nameOf(goal)).isEqualTo("3010");
  }

  private static List<Integer> neighboursOf(int vertex) {
    return Arrays.stream(map.neighborsOf(vertex)).boxed().toList();
  }
}
