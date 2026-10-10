package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.jupiter.api.Test;

class PlanBodyValidityTest {

  private static final double VEHICLE_LENGTH = 840.0;
  private static final double VEHICLE_WIDTH = 425.0;
  private static final double CARGO_LENGTH = 800.0;
  private static final double CARGO_WIDTH = 800.0;
  private static final double CLEARANCE = 50.0;

  private static final int INSTANCES = 20;
  private static final int AGENTS = 6;

  private static double halfAcrossX(int nose, boolean loaded) {
    double along = loaded ? Math.max(VEHICLE_LENGTH, CARGO_LENGTH) / 2 : VEHICLE_LENGTH / 2;
    double across = loaded ? Math.max(VEHICLE_WIDTH, CARGO_WIDTH) / 2 : VEHICLE_WIDTH / 2;
    if (nose == Roadmap.DIR_NONE) {
      return Math.hypot(along, across);
    }
    return nose == Roadmap.DIR_POS_X || nose == Roadmap.DIR_NEG_X ? along : across;
  }

  private static double halfAcrossY(int nose, boolean loaded) {
    double along = loaded ? Math.max(VEHICLE_LENGTH, CARGO_LENGTH) / 2 : VEHICLE_LENGTH / 2;
    double across = loaded ? Math.max(VEHICLE_WIDTH, CARGO_WIDTH) / 2 : VEHICLE_WIDTH / 2;
    if (nose == Roadmap.DIR_NONE) {
      return Math.hypot(along, across);
    }
    return nose == Roadmap.DIR_POS_Y || nose == Roadmap.DIR_NEG_Y ? along : across;
  }

  private static boolean bodiesOverlap(
      Roadmap map, int cellA, int noseA, boolean loadedA,
      int cellB, int noseB, boolean loadedB) {
    double dx = Math.abs(map.xOf(cellA) - map.xOf(cellB));
    double dy = Math.abs(map.yOf(cellA) - map.yOf(cellB));
    return dx < halfAcrossX(noseA, loadedA) + halfAcrossX(noseB, loadedB) + CLEARANCE
        && dy < halfAcrossY(noseA, loadedA) + halfAcrossY(noseB, loadedB) + CLEARANCE;
  }

  private static Roadmap load(String file) throws Exception {
    File found = RoadmapXmlLoader.findMap(file);
    return found == null ? null : new Roadmap(RoadmapXmlLoader.load(found));
  }

  private static int[] distinctReachableCells(Roadmap map, Random rng, int count) {
    List<Integer> usable = new ArrayList<>();
    for (int cell = 0; cell < map.size(); cell++) {
      if (map.neighborsOf(cell).length > 0) {
        usable.add(cell);
      }
    }
    int[] picked = new int[count];
    for (int i = 0; i < count; i++) {
      picked[i] = usable.remove(rng.nextInt(usable.size()));
    }
    return picked;
  }

  private static List<String> violationsIn(Roadmap map, String mapName, int instance) {
    Random rng = new Random(instance);
    int[] starts = distinctReachableCells(map, rng, AGENTS * 2);
    int[] goals = new int[AGENTS];
    int[] from = new int[AGENTS];
    boolean[] loaded = new boolean[AGENTS];
    int[] noses = new int[AGENTS];
    for (int a = 0; a < AGENTS; a++) {
      from[a] = starts[a];
      goals[a] = starts[AGENTS + a];
      loaded[a] = rng.nextBoolean();
      noses[a] = rng.nextBoolean() ? Roadmap.DIR_POS_X : Roadmap.DIR_POS_Y;
    }

    AgentPriority[] priorities = new AgentPriority[AGENTS];
    java.util.Arrays.fill(priorities, AgentPriority.NEWCOMER);
    Lacam2Solver.Result result = new Lacam2Solver(map).solve(
        new MapfInstance(from, goals), 1_000L, instance, Lacam2Solver.Objective.NONE,
        Lacam2Solver.DEFAULT_RESTART_RATE, Lacam2Solver.LOOP_LIMIT_UNBOUNDED,
        SolverParams.DEFAULT_WINDOW, priorities, noses, loaded, null, null,
        StaticBodies.NONE);

    List<String> violations = new ArrayList<>();
    if (!result.solved()) {
      return violations;
    }
    List<int[]> plan = result.solution();
    int[] nose = noses.clone();
    for (int t = 0; t < plan.size(); t++) {
      if (t > 0) {
        for (int a = 0; a < AGENTS; a++) {
          nose[a] = map.noseAfter(nose[a], plan.get(t - 1)[a], plan.get(t)[a]);
        }
      }
      for (int a = 0; a < AGENTS; a++) {
        for (int b = a + 1; b < AGENTS; b++) {
          if (bodiesOverlap(map, plan.get(t)[a], nose[a], loaded[a],
              plan.get(t)[b], nose[b], loaded[b])) {
            violations.add(String.format(
                "%s instance %d tick %d: a%d@%s(%s%s) overlaps a%d@%s(%s%s)",
                mapName, instance, t,
                a, map.nameOf(plan.get(t)[a]), nose[a], loaded[a] ? ",loaded" : "",
                b, map.nameOf(plan.get(t)[b]), nose[b], loaded[b] ? ",loaded" : ""));
          }
        }
      }
    }
    return violations;
  }

  private static void everyConfigurationMustBeBodyFree(String mapName) throws Exception {
    Roadmap map = load(mapName);
    assumeTrue(map != null, mapName + " not found; skipping");

    List<String> violations = new ArrayList<>();
    for (int instance = 0; instance < INSTANCES; instance++) {
      violations.addAll(violationsIn(map, mapName, instance));
    }
    System.out.printf("%-22s %d instances x %d agents: %d violating configurations%n",
        mapName, INSTANCES, AGENTS, violations.size());

    assertThat(violations)
        .as("the geometry here is written out longhand rather than calling Footprint, so a"
            + " mistake inside Footprint cannot hide by being made twice; every configuration"
            + " the solver emits must hold two bodies that do not overlap")
        .isEmpty();
  }

  @Test
  void everyConfigurationOnTheUniformMetreGridIsBodyFree() throws Exception {
    everyConfigurationMustBeBodyFree("v7-vda5050.xml");
  }

  @Test
  void everyConfigurationOnTheTightXuongGridIsBodyFree() throws Exception {
    everyConfigurationMustBeBodyFree("v7-map-xuong.xml");
  }
}
