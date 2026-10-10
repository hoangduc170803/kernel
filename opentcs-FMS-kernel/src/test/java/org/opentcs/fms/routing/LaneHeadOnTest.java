package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class LaneHeadOnTest {

  private static final String MAP_FILE = "v7-vda5050.xml";

  private static final long DEADLINE_MS = 1_000L;
  private static final int LOOP_LIMIT
      = Math.max(1, Integer.getInteger("fms.mapf.lacam2LoopLimit", 48_000));
  private static final int WINDOW = SolverParams.DEFAULT_WINDOW;
  private static final int[] SEEDS = { 0, 1, 2, 3, 4, 5, 6, 7 };

  private static final String LANE_MOUTH = "0086";
  private static final String OUTSIDE_THE_MOUTH = "0085";
  private static final String LANE_TIP = "0174";
  private static final String ONE_BEFORE_THE_TIP = "0164";
  private static final String MAINLINE_GOAL = "0004";
  private static final String RETREAT_CELL = "0084";
  private static final String MAINLINE_EAST_GOAL = "0091";

  private static final List<String> LANE_CHAIN = List.of(
      "0085", "0086", "0104", "0114", "0124", "0134", "0144", "0154", "0164", "0174");

  private static final String[] AGENTS = {
      "A wants in (0085 -> 0174)",
      "B wants out (0164 -> 0004)"
  };

  private static final int[] START_AXES = {
      Roadmap.DIR_POS_X, Roadmap.DIR_NEG_Y
  };

  private static final boolean[] BOTH_CARRY_A_LOAD = { true, true };

  private static Roadmap map;

  @BeforeAll
  static void loadTheProductionMap() throws Exception {
    File file = RoadmapXmlLoader.findMap(MAP_FILE);
    assumeTrue(file != null, MAP_FILE + " not found; skipping");
    map = new Roadmap(RoadmapXmlLoader.load(file));
  }

  @Test
  void thePickupLaneIsTheDeadEndCorridorTheProductionRouteWalked() {
    for (String point : LANE_CHAIN) {
      assertThat(map.has(point)).as("point %s is missing from %s", point, MAP_FILE).isTrue();
    }
    for (int i = 0; i + 1 < LANE_CHAIN.size(); i++) {
      String from = LANE_CHAIN.get(i);
      String to = LANE_CHAIN.get(i + 1);
      assertThat(map.neighborsOf(map.idOf(from)))
          .as("%s -> %s must be walkable", from, to)
          .contains(map.idOf(to));
      assertThat(map.neighborsOf(map.idOf(to)))
          .as("%s -> %s must be walkable (the ~rev edge)", to, from)
          .contains(map.idOf(from));
    }
    for (String interior : LANE_CHAIN.subList(2, LANE_CHAIN.size() - 1)) {
      assertThat(map.neighborsOf(map.idOf(interior)))
          .as("%s must be a plain corridor cell, so the two agents cannot pass", interior)
          .hasSize(2);
    }
    assertThat(map.neighborsOf(map.idOf(LANE_TIP)))
        .as("%s must be the dead end of the lane", LANE_TIP)
        .hasSize(1);
    assertThat(map.neighborsOf(map.idOf(LANE_MOUTH)))
        .as("%s must be the mainline junction that feeds the lane", LANE_MOUTH)
        .hasSize(4);
  }

  @Test
  void neitherAgentRetreatsAndReturnsWhenTheyMeetHeadOnInThePickupLane() {
    MapfInstance headOn = new MapfInstance(
        new int[] { map.idOf(OUTSIDE_THE_MOUTH), map.idOf(ONE_BEFORE_THE_TIP) },
        new int[] { map.idOf(LANE_TIP), map.idOf(MAINLINE_GOAL) });

    List<String> defects = new ArrayList<>();
    long startedAt = System.nanoTime();
    for (int seed : SEEDS) {
      Lacam2Solver.Result result = solve(headOn, seed);
      for (int agent = 0; agent < AGENTS.length; agent++) {
        List<String> raw = rawRouteOf(result, agent);
        List<String> compressed = compress(raw);
        String repeated = firstVertexVisitedTwice(compressed);
        if (repeated != null) {
          defects.add(String.format(
              "seed=%d %s revisits %s%n      raw        = %s%n      compressed = %s",
              seed, AGENTS[agent], repeated, raw, compressed));
        }
      }
      System.out.printf(
          "[LaneHeadOn] seed=%d solved=%s reachedGoals=%s makespan=%d loops=%d nodes=%d %dms%n"
              + "    A raw=%s compressed=%s%n    B raw=%s compressed=%s%n",
          seed, result.solved(), result.reachedGoals(), result.solution().size() - 1,
          result.loopCount(), result.nodesGenerated(), result.runtimeMs(),
          rawRouteOf(result, 0), compress(rawRouteOf(result, 0)),
          rawRouteOf(result, 1), compress(rawRouteOf(result, 1)));
    }
    long elapsedMs = (System.nanoTime() - startedAt) / 1_000_000L;
    System.out.printf(
        "[LaneHeadOn] %d seeds x %d agents, window=%d deadline=%dms loopLimit=%d"
            + " -> %d defective agent plans in %dms wall clock%n",
        SEEDS.length, AGENTS.length, WINDOW, DEADLINE_MS, LOOP_LIMIT, defects.size(), elapsedMs);

    assertThat(defects)
        .as("a compressed route that stands on the same point twice makes openTCS count that"
            + " point's reservation twice; production emitted 0086 -> 0085 -> 0086 on the way"
            + " into this lane")
        .isEmpty();
  }

  @Test
  void theInboundVehicleBacksOffTwoCellsWhenTheOutboundOneTurnsAtTheMouth() {
    MapfInstance turningExit = new MapfInstance(
        new int[] { map.idOf(OUTSIDE_THE_MOUTH), map.idOf(ONE_BEFORE_THE_TIP) },
        new int[] { map.idOf(LANE_TIP), map.idOf(MAINLINE_EAST_GOAL) });

    List<String> defects = new ArrayList<>();
    for (int seed : SEEDS) {
      Lacam2Solver.Result result = solve(turningExit, seed);
      List<String> inbound = rawRouteOf(result, 0);
      List<String> outbound = rawRouteOf(result, 1);
      System.out.printf(
          "[MouthTurn] seed=%d makespan=%d%n    A raw=%s%n    B raw=%s%n",
          seed, result.solution().size() - 1, inbound, outbound);

      int mouthTurn = stepWhereTheOutboundVehicleTurnsAtTheMouth(outbound);
      if (mouthTurn < 0) {
        continue;
      }
      if (OUTSIDE_THE_MOUTH.equals(inbound.get(mouthTurn))) {
        defects.add(String.format(
            "seed=%d A stands on %s at t=%d while B pivots at %s -- a loaded swept disc reaches"
                + " 580mm and a resting load 400mm, so 950mm apart is 80mm short%n      A = %s"
                + "%n      B = %s",
            seed, OUTSIDE_THE_MOUTH, mouthTurn, LANE_MOUTH, inbound, outbound));
      }
    }

    assertThat(defects)
        .as("%s is the only cell within one hop of %s that clears the sweep, so the inbound"
            + " vehicle has to give up %s and wait on %s",
            RETREAT_CELL, LANE_MOUTH, OUTSIDE_THE_MOUTH, RETREAT_CELL)
        .isEmpty();
  }

  private static int stepWhereTheOutboundVehicleTurnsAtTheMouth(List<String> outbound) {
    for (int t = 0; t + 1 < outbound.size(); t++) {
      if (LANE_MOUTH.equals(outbound.get(t))
          && !LANE_MOUTH.equals(outbound.get(t + 1))
          && !"0104".equals(outbound.get(t + 1))
          && !"0009".equals(outbound.get(t + 1))) {
        return t;
      }
    }
    return -1;
  }

  private static Lacam2Solver.Result solve(MapfInstance instance, int seed) {
    return new Lacam2Solver(map).solve(
        instance,
        DEADLINE_MS,
        seed,
        Lacam2Solver.Objective.SUM_OF_LOSS,
        Lacam2Solver.DEFAULT_RESTART_RATE,
        LOOP_LIMIT,
        WINDOW,
        null,
        START_AXES,
        BOTH_CARRY_A_LOAD,
        null,
        null,
        StaticBodies.NONE);
  }

  private static List<String> rawRouteOf(Lacam2Solver.Result result, int agent) {
    List<String> route = new ArrayList<>();
    for (int[] config : result.solution()) {
      route.add(map.nameOf(config[agent]));
    }
    return route;
  }

  private static List<String> compress(List<String> raw) {
    List<String> compressed = new ArrayList<>();
    for (String point : raw) {
      if (compressed.isEmpty() || !compressed.get(compressed.size() - 1).equals(point)) {
        compressed.add(point);
      }
    }
    return compressed;
  }

  private static String firstVertexVisitedTwice(List<String> compressed) {
    for (int i = 0; i < compressed.size(); i++) {
      for (int j = i + 1; j < compressed.size(); j++) {
        if (compressed.get(i).equals(compressed.get(j))) {
          return compressed.get(i);
        }
      }
    }
    return null;
  }
}
