package org.opentcs.fms.kernel.mapf;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import java.util.Map;
import java.util.Set;

import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.Roadmap;

class SolverInputTransformerTest {

  private final SolverInputTransformer transformer = new SolverInputTransformer();

  @Test
  void retreatGoalsAreConvertedUsingTheAgentOrder() {
    Roadmap roadmap = RoadmapFixtures.columnRoadmap("A", "B", "C");
    ClaimPolicy policy = new ClaimPolicy(
        Map.of(),
        Set.of("V1", "V3"),
        Set.of(),
        Set.of(),
        Set.of(),
        Map.of(),
        Map.of("V1", "C", "V3", "missing"));

    assertThat(transformer.retreatGoals(List.of("V1", "V2", "V3"), policy, roadmap))
        .containsExactly(roadmap.idOf("C"), -1, -1);
  }

}
