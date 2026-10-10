package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.pibt.Footprint;

class LocalGuideTest {

  private static final int CORRIDOR_LENGTH = 6;
  private static final String POCKET = "pocket";
  private static final int SEED = 0;
  private static final int BLOCKER = 1;
  private static final int GUIDED_AGENT = 0;
  private static final int LAST_BLOCKED_TIME = 2;

  private static final List<String> GUIDE_PROPERTIES = List.of(
      "fms.guide.on",
      "fms.guide.numRefine",
      "fms.guide.deterministic",
      "fms.guide.collisionCost",
      "fms.guide.window",
      "fms.lllg.window",
      "fms.lllg.numRefine",
      "fms.lllg.deterministic");

  private static final String PRODUCTION_MAP = "v7-vda5050-test.xml";
  private static final String CHARGER_ONLY_MAP = "v7-vda5050.xml";
  private static final String OUTSIDE_THE_MOUTH = "0085";
  private static final String LANE_MOUTH = "0086";
  private static final String LANE_TIP = "0174";
  private static final String ONE_BEFORE_THE_TIP = "0164";
  private static final String MAINLINE_GOAL = "0004";

  private static final List<String> LANE_CHAIN = List.of(
      "0085", "0086", "0104", "0114", "0124", "0134", "0144", "0154", "0164", "0174");

  private static final List<String> LANE_CELLS_WITH_NO_PASSING_PLACE = List.of(
      "0104", "0114", "0124", "0134", "0144", "0154", "0164", "0174");

  private static final int WANTS_IN = 0;
  private static final int WANTS_OUT = 1;
  private static final String[] AGENT_LABELS = {
      "A in  0085->0174",
      "B out 0164->0004"
  };

  private static final int[] INSIDER_PLANS_LAST = { WANTS_IN, WANTS_OUT };
  private static final int[] INSIDER_PLANS_FIRST = { WANTS_OUT, WANTS_IN };
  private static final List<int[]> PLANNING_ORDERS
      = List.of(INSIDER_PLANS_LAST, INSIDER_PLANS_FIRST);

  private static final String[] COLLISION_COSTS = { "2.0", "4.0", "8.0", "16.0" };
  private static final String[] REFINE_ROUNDS = { "0", "1", "2" };
  private static final String DEFAULT_COLLISION_COST = "3.0";
  private static final String PINNED_REFINE_ROUNDS = "2";
  private static final int[] TIE_BREAK_SEEDS = { 0, 1, 2, 3, 4, 5, 6, 7 };

  private static final String APRON_BEHIND = "0087";
  private static final String APRON_FAR = "0088";
  private static final String LEADER_GOAL = "3127";
  private static final String FOLLOWER_GOAL = "3126";

  private static final int LEADER = 0;
  private static final int FOLLOWER = 1;
  private static final String[] QUEUE_LABELS = {
      "leader   0086->3127",
      "follower 0087->3126"
  };
  private static final int[] LEADER_FIRST = { LEADER, FOLLOWER };
  private static final int[] FOLLOWER_FIRST = { FOLLOWER, LEADER };
  private static final List<int[]> QUEUE_ORDERS = List.of(LEADER_FIRST, FOLLOWER_FIRST);
  private static final int QUEUE_TICKS = 8;

  private static final int WAITING = 0;
  private static final int OUTBOUND = 1;
  private static final int[] WAITING_FIRST = { WAITING, OUTBOUND };

  private static final String[] APRON_STARTS = {
      "0115", "3121", "0137", "0124", "3069", "0085", "3093", "3152", "0008", "3057", "3081"
  };
  private static final String[] APRON_GOALS = {
      "0165", "0169", "0167", "3145", "3115", "3133", "3095", "0168", "0166", "3173", "0170"
  };
  private static final boolean[] APRON_LOADED = {
      false, false, false, true, true, true, true, false, false, true, false
  };
  private static final int[] APRON_ORDER = { 4, 6, 9, 5, 8, 0, 2, 3, 7, 1, 10 };
  private static final String[] JUNCTION_STARTS = { "0115", "0087" };
  private static final String[] JUNCTION_GOALS = { "0091", "0175" };
  private static final boolean[] JUNCTION_LOADED = { true, true };
  private static final int[] JUNCTION_ORDER = { 0, 1 };

  private static final int APRON_TICKS = 6;
  private static final int APRON_SUBJECT = 5;
  private static final String APRON_SUBJECT_NAME = "Vehicle-0006";

  private static final int NEVER = -1;

  private static Roadmap cachedProductionMap;
  private static boolean productionMapLookedUp;

  @AfterEach
  void restoreGuideDefaults() {
    GUIDE_PROPERTIES.forEach(System::clearProperty);
  }

  @Test
  void inventsAWaitWhenTheCorridorIsHeldByAnEnrolledPath() {
    Roadmap corridor = corridorWithSidePocket();
    int entry = corridor.idOf(cell(0));
    int held = corridor.idOf(cell(2));
    int exit = corridor.idOf(cell(CORRIDOR_LENGTH - 1));
    int pocket = corridor.idOf(POCKET);

    int[] goals = { exit, pocket };
    int[] configuration = { entry, held };
    int[] order = { GUIDED_AGENT, BLOCKER };
    DistTable dist = DistTable.turnAware(corridor, goals);

    System.setProperty("fms.guide.on", "true");
    System.setProperty("fms.guide.numRefine", "1");
    System.setProperty("fms.guide.deterministic", "true");

    FmsLocalGuide overAClearCorridor = FmsLocalGuide.builder(corridor, dist, goals, SEED).build();
    overAClearCorridor.construct(configuration, order);
    int[] clearRun = overAClearCorridor.currentGuidePaths()[GUIDED_AGENT];

    FmsLocalGuide behindTheBlocker = FmsLocalGuide.builder(corridor, dist, goals, SEED).build();
    behindTheBlocker.setGuidePaths(new int[][] {
        null,
        { held, held, held, held, pocket }
    });
    behindTheBlocker.construct(configuration, order);
    int[] blockedRun = behindTheBlocker.currentGuidePaths()[GUIDED_AGENT];

    assertThat(blockedRun[0])
        .as("the guide path must start where the agent stands")
        .isEqualTo(entry);
    assertThat(waitStepsBeforeArrival(clearRun, exit))
        .as("clear corridor, guide path %s", render(corridor, clearRun, exit))
        .isZero();
    assertThat(waitStepsBeforeArrival(blockedRun, exit))
        .as("corridor held until t=%d, guide path %s",
            LAST_BLOCKED_TIME, render(corridor, blockedRun, exit))
        .isGreaterThanOrEqualTo(1);
    assertThat(firstTimeAt(blockedRun, held))
        .as("entered %s while the enrolled path still occupies it, guide path %s",
            corridor.nameOf(held), render(corridor, blockedRun, exit))
        .isGreaterThan(LAST_BLOCKED_TIME);
  }

  @Test
  void preservesTheCommittedCurrentToFrontierEdgeBeforePlanningItsTail() {
    Roadmap lane = GridFixture.grid(1, 3);
    int current = GridFixture.id(lane, 0, 0);
    int frontier = GridFixture.id(lane, 1, 0);
    int goal = GridFixture.id(lane, 2, 0);

    System.setProperty("fms.guide.on", "true");
    System.setProperty("fms.guide.window", "5");
    System.setProperty("fms.guide.numRefine", "1");
    System.setProperty("fms.guide.deterministic", "true");

    FmsLocalGuide guide = FmsLocalGuide.builder(
        lane,
        DistTable.turnAware(lane, new int[] {goal}),
        new int[] {goal},
        SEED
    )
        .footprint(null)
        .loaded(null)
        .committedPrefixes(new int[][] {{current, frontier}})
        .build();
    guide.construct(new int[] {current}, new int[] {Roadmap.DIR_NONE}, null, new int[] {0}, 0);

    assertThat(guide.currentGuidePaths()[0])
        .as("the guide contains the real current position and its sent first edge before its tail")
        .containsExactly(current, frontier, goal, goal, goal);
    assertThat(guide.guidedConfiguration()[0])
        .as("PIBT is guided to the already-sent frontier, not allowed to choose a replacement")
        .isEqualTo(frontier);
  }

  @Test
  void plansAroundTheContinuingWarmStartOfAnActivePrefix() {
    Roadmap lane = GridFixture.grid(1, 7);
    int aStart = GridFixture.id(lane, 6, 0);
    int aGoal = GridFixture.id(lane, 0, 0);
    int bPrevious = GridFixture.id(lane, 1, 0);
    int bCurrent = GridFixture.id(lane, 2, 0);
    int bNext = GridFixture.id(lane, 3, 0);
    int bTailAtTickTwo = GridFixture.id(lane, 4, 0);
    int bGoal = GridFixture.id(lane, 5, 0);
    int[] starts = {aStart, bCurrent};
    int[] goals = {aGoal, bGoal};

    System.setProperty("fms.guide.on", "true");
    System.setProperty("fms.guide.window", "7");
    System.setProperty("fms.guide.numRefine", "1");
    System.setProperty("fms.guide.deterministic", "true");

    FmsLocalGuide guide = FmsLocalGuide.builder(
        lane,
        DistTable.turnAware(lane, goals),
        goals,
        SEED
    )
        .committedPrefixes(new int[][] {null, {bCurrent, bNext}})
        .build();
    guide.setGuidePaths(new int[][] {
        null,
        {bPrevious, bCurrent, bNext, bTailAtTickTwo, bGoal, bGoal, bGoal}
    });
    guide.construct(starts, new int[] {GUIDED_AGENT, BLOCKER});

    int[][] paths = guide.currentGuidePaths();
    assertThat(paths[BLOCKER]).contains(bCurrent, bNext, bTailAtTickTwo);
    assertThat(paths[GUIDED_AGENT][2])
        .as("agent A must not enter B's warm-start cell at tick 2")
        .isNotEqualTo(bTailAtTickTwo);
  }

  @Test
  void restoresTheParentGuideBeforeAdvancingTheCommittedPrefix() {
    Roadmap lane = GridFixture.grid(1, 4);
    int current = GridFixture.id(lane, 0, 0);
    int firstCommitted = GridFixture.id(lane, 1, 0);
    int secondCommitted = GridFixture.id(lane, 2, 0);
    int goal = GridFixture.id(lane, 3, 0);
    int[][] committed = {{current, firstCommitted, secondCommitted}};

    System.setProperty("fms.guide.on", "true");
    System.setProperty("fms.guide.window", "5");
    System.setProperty("fms.guide.numRefine", "1");
    System.setProperty("fms.guide.deterministic", "true");

    FmsLocalGuide guide = FmsLocalGuide.builder(
        lane,
        DistTable.turnAware(lane, new int[] {goal}),
        new int[] {goal},
        SEED
    )
        .footprint(null)
        .loaded(null)
        .committedPrefixes(committed)
        .build();
    guide.construct(
        new int[] {current}, new int[] {Roadmap.DIR_NONE}, null, new int[] {0}, 0);
    int[][] parentGuide = guide.currentGuidePaths();
    guide.setGuidePaths(new int[][] {{current, current, current}});
    guide.setGuidePaths(parentGuide);

    guide.construct(
        new int[] {firstCommitted}, new int[] {Roadmap.DIR_POS_X},
        new int[] {Roadmap.DIR_POS_X}, new int[] {0}, 1);

    assertThat(guide.currentGuidePaths()[0])
        .containsExactly(firstCommitted, secondCommitted, goal, goal, goal);
    assertThat(guide.guidedConfiguration()[0]).isEqualTo(secondCommitted);
  }

  @Test
  void advancesThePreviousPlanSeedByTheExecutedStepAndPadsItsEndpoint() {
    Roadmap corridor = corridorWithSidePocket();
    int previousStart = corridor.idOf(cell(0));
    int current = corridor.idOf(cell(1));
    int next = corridor.idOf(cell(2));
    int goal = corridor.idOf(cell(CORRIDOR_LENGTH - 1));

    System.setProperty("fms.guide.on", "true");
    System.setProperty("fms.guide.window", "5");
    System.setProperty("fms.guide.numRefine", "0");
    System.setProperty("fms.guide.deterministic", "true");

    FmsLocalGuide guide = FmsLocalGuide.builder(
        corridor,
        DistTable.turnAware(corridor, new int[] {goal}),
        new int[] {goal},
        SEED
    )
        .build();
    guide.setGuidePaths(new int[][] {{previousStart, current, next}});
    assertThat(guide.guidedConfiguration()[GUIDED_AGENT]).isEqualTo(current);
    guide.construct(new int[] {current}, new int[] {GUIDED_AGENT});

    assertThat(guide.counters().warmStartSlides).isEqualTo(1);
    assertThat(guide.currentGuidePaths()[GUIDED_AGENT])
        .containsExactly(current, next, next, next, next);
  }

  @Test
  void matchesLllgByStoppingCostAtTheGoal() {
    Roadmap lane = GridFixture.grid(1, 6);
    int nearStart = GridFixture.id(lane, 0, 0);
    int nearGoal = GridFixture.id(lane, 1, 0);
    int farGoal = GridFixture.id(lane, 0, 0);
    int farCurrent = GridFixture.id(lane, 4, 0);

    System.setProperty("fms.guide.on", "true");
    System.setProperty("fms.guide.window", "5");
    System.setProperty("fms.guide.numRefine", "0");
    System.setProperty("fms.guide.deterministic", "true");
    System.setProperty("fms.lllg.window", "5");
    System.setProperty("fms.lllg.numRefine", "0");
    System.setProperty("fms.lllg.deterministic", "true");

    int[] goals = {nearGoal, farGoal};
    int[][] previousPaths = {
        null,
        {
            GridFixture.id(lane, 5, 0), farCurrent, GridFixture.id(lane, 3, 0),
            GridFixture.id(lane, 2, 0), nearGoal
        }
    };
    int[] configuration = {nearStart, farCurrent};
    int[] order = {1, 0};

    LllgLocalGuide baseline = new LllgLocalGuide(
        lane, DistTable.weighted(lane, goals), goals, goals.length, SEED);
    baseline.setGuidePaths(previousPaths);
    baseline.construct(configuration, null, order, 0);

    FmsLocalGuide guide = FmsLocalGuide.builder(
        lane,
        DistTable.turnAware(lane, goals),
        goals,
        SEED
    )
        .build();
    guide.setGuidePaths(previousPaths);
    guide.construct(configuration, order);

    assertThat(guide.currentGuidePaths()[0])
        .as("reaching the current goal completes the task before a later planning cycle assigns"
            + " the next goal")
        .isEqualTo(baseline.currentGuidePaths()[0])
        .containsExactly(nearStart, nearGoal, nearGoal, nearGoal, nearGoal);
  }

  @Test
  void theTestMapCarriesTheCargoSlotsThatTheChargerOnlyMapLacks() throws Exception {
    Roadmap withCargoSlots = productionMap();
    assumeTrue(withCargoSlots != null, PRODUCTION_MAP + " not found; skipping");

    for (String point : LANE_CHAIN) {
      assertThat(withCargoSlots.has(point))
          .as("point %s is missing from %s", point, PRODUCTION_MAP)
          .isTrue();
    }
    for (int i = 0; i + 1 < LANE_CHAIN.size(); i++) {
      String from = LANE_CHAIN.get(i);
      String to = LANE_CHAIN.get(i + 1);
      assertThat(withCargoSlots.neighborsOf(withCargoSlots.idOf(from)))
          .as("%s -> %s must be walkable", from, to)
          .contains(withCargoSlots.idOf(to));
      assertThat(withCargoSlots.neighborsOf(withCargoSlots.idOf(to)))
          .as("%s -> %s must be walkable (the ~rev edge)", to, from)
          .contains(withCargoSlots.idOf(from));
    }

    System.out.printf(
        "[FmsLocalGuide/map] %s points=%d maxVertexWeight=%d%n",
        PRODUCTION_MAP, withCargoSlots.size(), withCargoSlots.maxVertexWeight());
    for (String point : LANE_CHAIN) {
      System.out.printf(
          "    %s weight=%d degree=%d%n",
          point,
          withCargoSlots.weightOf(withCargoSlots.idOf(point)),
          withCargoSlots.neighborsOf(withCargoSlots.idOf(point)).length);
    }

    File chargerOnly = RoadmapXmlLoader.findMap(CHARGER_ONLY_MAP);
    if (chargerOnly != null) {
      Roadmap withoutCargoSlots = new Roadmap(RoadmapXmlLoader.load(chargerOnly));
      System.out.printf(
          "[FmsLocalGuide/map] %s points=%d maxVertexWeight=%d laneTipWeight=%d%n",
          CHARGER_ONLY_MAP,
          withoutCargoSlots.size(),
          withoutCargoSlots.maxVertexWeight(),
          withoutCargoSlots.weightOf(withoutCargoSlots.idOf(LANE_TIP)));
      assertThat(withoutCargoSlots.maxVertexWeight())
          .as("%s instantiates chargers only, so no vertex carries the cargo big-M",
              CHARGER_ONLY_MAP)
          .isEqualTo(1);
    }

    assertThat(withCargoSlots.neighborsOf(withCargoSlots.idOf(LANE_MOUTH)))
        .as("%s must be the only mouth feeding the lane", LANE_MOUTH)
        .hasSize(4);
    assertThat(withCargoSlots.neighborsOf(withCargoSlots.idOf(LANE_TIP)))
        .as("%s must be the dead end, so the two agents cannot pass inside", LANE_TIP)
        .hasSize(1);
    assertThat(withCargoSlots.weightOf(withCargoSlots.idOf(LANE_TIP)))
        .as("%s must be a cargo slot in %s", LANE_TIP, PRODUCTION_MAP)
        .isGreaterThan(1);
    assertThat(withCargoSlots.weightOf(withCargoSlots.idOf(OUTSIDE_THE_MOUTH)))
        .as("%s is outside the lane, so it must stay at uniform weight", OUTSIDE_THE_MOUTH)
        .isEqualTo(1);
  }

  @Test
  void reportsThePlanningOrderTimesRefineRoundsGrid() throws Exception {
    Roadmap map = productionMap();
    assumeTrue(map != null, PRODUCTION_MAP + " not found; skipping");

    System.out.printf(
        "[FmsLocalGuide/grid] map=%s collisionCost=%s hops(A)=%d hops(B)=%d%n",
        PRODUCTION_MAP, DEFAULT_COLLISION_COST,
        hopsBetween(map, OUTSIDE_THE_MOUTH, LANE_TIP),
        hopsBetween(map, ONE_BEFORE_THE_TIP, MAINLINE_GOAL));
    for (int[] order : PLANNING_ORDERS) {
      for (String refineRounds : REFINE_ROUNDS) {
        enableGuide(DEFAULT_COLLISION_COST, refineRounds);
        FmsLocalGuide guide = guideOverTheHeadOn(map, order);
        printHeadOn(
            map,
            String.format("order=%s numRefine=%s", labelOf(order), refineRounds),
            guide);
      }
    }
  }

  @Test
  void reportsTheCollisionCostSweep() throws Exception {
    Roadmap map = productionMap();
    assumeTrue(map != null, PRODUCTION_MAP + " not found; skipping");

    System.out.printf(
        "[FmsLocalGuide/sweep] %-6s %-6s %-7s %-8s %-10s %-9s %-9s %-9s %-9s %-10s %s%n",
        "cost", "order", "refine", "A waits", "A revisit", "A arrive",
        "A enters", "B clears", "B revisit", "B charge", "headOn");
    for (String collisionCost : COLLISION_COSTS) {
      for (int[] order : PLANNING_ORDERS) {
        for (String refineRounds : REFINE_ROUNDS) {
          enableGuide(collisionCost, refineRounds);
          FmsLocalGuide guide = guideOverTheHeadOn(map, order);
          HeadOn outcome = analyse(map, guide);
          System.out.printf(
              "[FmsLocalGuide/sweep] %-6s %-6s %-7s %-8d %-10s %-9s %-9s %-9s %-9s %-10.1f %s%n",
              collisionCost, labelOf(order), refineRounds,
              outcome.wantsIn().waitSteps(),
              orNone(outcome.wantsIn().revisit()),
              orNever(outcome.wantsIn().arrivalStep()),
              orNever(outcome.firstStepAgentAinTheLane()),
              orNever(outcome.lastStepAgentBinTheLane()),
              orNone(outcome.wantsOut().revisit()),
              guide.collisionCostOf(WANTS_OUT),
              outcome.headOnStep() == NEVER
                  ? "feasible"
                  : "HEAD-ON at t=" + outcome.headOnStep());
        }
      }
    }
  }

  @Test
  void reportsWhetherTheBackOffToTheMouthApronIsForcedOrATie() throws Exception {
    Roadmap map = productionMap();
    assumeTrue(map != null, PRODUCTION_MAP + " not found; skipping");

    enableGuide(DEFAULT_COLLISION_COST, PINNED_REFINE_ROUNDS);
    FmsLocalGuide guide = guideOverTheHeadOn(map, INSIDER_PLANS_LAST);
    HeadOn outcome = analyse(map, guide);
    List<String> chosen = outcome.wantsIn().full();
    List<String> standingOffOutside = standOffOutsideTheMouthUntil(chosen.size());
    List<String> divingOutAndBack = divertIntoTheMouthAndBackUntil(chosen.size());

    System.out.printf(
        "[FmsLocalGuide/tie] chosen        cost=%d moves=%d arrival=%s revisit=%s%n        %s%n",
        weightedCostOf(map, chosen), movesIn(chosen), orNever(outcome.wantsIn().arrivalStep()),
        orNone(outcome.wantsIn().revisit()), chosen);
    System.out.printf(
        "[FmsLocalGuide/tie] standingOff   cost=%d moves=%d arrival=%s revisit=%s headOn=%s%n"
            + "        %s%n",
        weightedCostOf(map, standingOffOutside), movesIn(standingOffOutside),
        orNever(standingOffOutside.indexOf(LANE_TIP)),
        orNone(firstRepeat(compress(standingOffOutside))),
        stepOfFirstLaneCellBefore(
            standingOffOutside, outcome.lastStepAgentBinTheLane()) == NEVER
            ? "feasible"
            : "HEAD-ON",
        standingOffOutside);
    System.out.printf(
        "[FmsLocalGuide/tie] divingOutAndBack cost=%d moves=%d arrival=%s revisit=%s%n        %s%n",
        weightedCostOf(map, divingOutAndBack), movesIn(divingOutAndBack),
        orNever(divingOutAndBack.indexOf(LANE_TIP)),
        orNone(firstRepeat(compress(divingOutAndBack))),
        divingOutAndBack);
  }

  @Test
  void reportsTieBreakSensitivityAcrossSeeds() throws Exception {
    Roadmap map = productionMap();
    assumeTrue(map != null, PRODUCTION_MAP + " not found; skipping");

    for (String collisionCost : COLLISION_COSTS) {
      for (int seed : TIE_BREAK_SEEDS) {
        System.setProperty("fms.guide.on", "true");
        System.setProperty("fms.guide.deterministic", "false");
        System.setProperty("fms.guide.numRefine", PINNED_REFINE_ROUNDS);
        System.setProperty("fms.guide.collisionCost", collisionCost);
        FmsLocalGuide guide = guideOverTheHeadOn(map, INSIDER_PLANS_LAST, seed);
        HeadOn outcome = analyse(map, guide);
        System.out.printf(
            "[FmsLocalGuide/seeds] cost=%-5s seed=%d A(waits=%d arrival=%s revisit=%s)"
                + " B(revisit=%s) %s%n",
            collisionCost, seed,
            outcome.wantsIn().waitSteps(),
            orNever(outcome.wantsIn().arrivalStep()),
            orNone(outcome.wantsIn().revisit()),
            orNone(outcome.wantsOut().revisit()),
            outcome.headOnStep() == NEVER
                ? "feasible"
                : "HEAD-ON at t=" + outcome.headOnStep());
      }
    }
  }

  @Test
  void theInboundAgentNeverRevisitsACellItHasAlreadyLeft() throws Exception {
    Roadmap map = productionMap();
    assumeTrue(map != null, PRODUCTION_MAP + " not found; skipping");

    enableGuide(DEFAULT_COLLISION_COST, PINNED_REFINE_ROUNDS);
    GuidedRoute wantsIn = analyse(map, guideOverTheHeadOn(map, INSIDER_PLANS_LAST)).wantsIn();

    assertThat(wantsIn.revisit())
        .as("collisionCost=%s numRefine=%s order=%s seed=%d: openTCS counts the reservation of a"
            + " revisited point twice and never releases it, so the compressed guide path of %s"
            + " may not repeat a vertex; a run of the same vertex is a wait, not a revisit"
            + "%n      compressed = %s%n      full       = %s",
            DEFAULT_COLLISION_COST, PINNED_REFINE_ROUNDS, labelOf(INSIDER_PLANS_LAST), SEED,
            AGENT_LABELS[WANTS_IN], wantsIn.compressed(), wantsIn.full())
        .isNull();
    assertThat(wantsIn.compressed())
        .as("compressed guide path of %s at collisionCost=%s numRefine=%s order=%s seed=%d",
            AGENT_LABELS[WANTS_IN], DEFAULT_COLLISION_COST, PINNED_REFINE_ROUNDS,
            labelOf(INSIDER_PLANS_LAST), SEED)
        .doesNotHaveDuplicates();
  }

  @Test
  void theInboundAgentStaysOutOfTheLaneUntilTheOutboundAgentHasClearedIt() throws Exception {
    Roadmap map = productionMap();
    assumeTrue(map != null, PRODUCTION_MAP + " not found; skipping");

    List<String> headOns = new ArrayList<>();
    for (int[] order : PLANNING_ORDERS) {
      enableGuide(DEFAULT_COLLISION_COST, PINNED_REFINE_ROUNDS);
      HeadOn outcome = analyse(map, guideOverTheHeadOn(map, order));
      if (outcome.headOnStep() != NEVER) {
        headOns.add(String.format(
            "order=%s meets head-on at t=%d, A enters the lane at t=%d but B is still inside"
                + " until t=%d%n      A = %s%n      B = %s",
            labelOf(order), outcome.headOnStep(),
            outcome.firstStepAgentAinTheLane(), outcome.lastStepAgentBinTheLane(),
            outcome.wantsIn().full(), outcome.wantsOut().full()));
      }
    }

    assertThat(headOns)
        .as("%s is a dead end and %s..%s has no passing place, so the guide path of the"
            + " inbound agent may not touch a lane cell while the outbound agent is still"
            + " inside; numRefine=%s is what lets the inbound agent see the outbound one",
            LANE_TIP, LANE_CELLS_WITH_NO_PASSING_PLACE.get(0),
            LANE_TIP, PINNED_REFINE_ROUNDS)
        .isEmpty();
  }

  @Test
  void theRightOfWayVehicleAtTheMouthNeverBacksDownTheApron() throws Exception {
    Roadmap map = productionMap();
    assumeTrue(map != null, PRODUCTION_MAP + " not found; skipping");

    List<String> backedOff = new ArrayList<>();
    for (int[] order : QUEUE_ORDERS) {
      for (int seed : TIE_BREAK_SEEDS) {
        enableGuideAsProduction();
        FmsLocalGuide guide = guideOverTheQueue(map, order, seed);
        int served = order[0];
        GuidedRoute route = measure(map, guide.currentGuidePaths()[served], queueGoals(map)[served]);
        if (route.revisit() != null) {
          backedOff.add(String.format(
              "order=%s seed=%d %s leaves and re-enters %s%n      compressed = %s",
              queueLabelOf(order), seed, QUEUE_LABELS[served], route.revisit(),
              route.compressed()));
        }
      }
    }

    assertThat(backedOff)
        .as("the apron is a straight row whose only turn into the drop-off lane is at %s; the"
            + " vehicle served first holds the right of way, so its guide path may never step"
            + " back along the apron and return", LANE_MOUTH)
        .isEmpty();
  }

  @Test
  void theQueuedPairYieldsOnceInsteadOfTradingWhoBacksOff() throws Exception {
    Roadmap map = productionMap();
    assumeTrue(map != null, PRODUCTION_MAP + " not found; skipping");

    List<String> wobbles = new ArrayList<>();
    for (int[] order : QUEUE_ORDERS) {
      for (int seed : TIE_BREAK_SEEDS) {
        enableGuideAsProduction();
        List<List<String>> tracks = driveTheQueue(map, order, seed);
        String[] goalNames = { LEADER_GOAL, FOLLOWER_GOAL };
        for (int slot = 0; slot < order.length; slot++) {
          int agent = order[slot];
          List<String> track = tracks.get(agent);
          int allowedVisits = slot == 0 ? 1 : 2;
          String worn = cellVisitedMoreThan(track, allowedVisits);
          if (worn != null) {
            wobbles.add(String.format(
                "order=%s seed=%d %s sits on %s more than %d time(s)%n      track = %s",
                queueLabelOf(order), seed, QUEUE_LABELS[agent], worn, allowedVisits, track));
          }
          int startHops = hopsBetween(map, track.get(0), goalNames[agent]);
          int endHops = hopsBetween(map, track.get(track.size() - 1), goalNames[agent]);
          if (endHops >= startHops) {
            wobbles.add(String.format(
                "order=%s seed=%d %s ends %d hops out, no closer than the %d it started at%n"
                    + "      track = %s",
                queueLabelOf(order), seed, QUEUE_LABELS[agent], endHops, startHops, track));
          }
        }
      }
    }

    assertThat(wobbles)
        .as("each solve builds a fresh guide seeded with the previous plan, as MapfPlanner does;"
            + " over %d solves the vehicle served first may never give ground and the one served"
            + " second may step aside exactly once, and both must end nearer their goal",
            QUEUE_TICKS)
        .isEmpty();
  }

  private static String cellVisitedMoreThan(List<String> track, int times) {
    List<String> compressed = compress(track);
    for (String cell : compressed) {
      int seen = 0;
      for (String other : compressed) {
        if (other.equals(cell)) {
          seen++;
        }
      }
      if (seen > times) {
        return cell;
      }
    }
    return null;
  }

  @Test
  void theVehicleAtTheApronWaitsInPlaceInsteadOfSteppingOutAndBack() throws Exception {
    Roadmap map = productionMap();
    assumeTrue(map != null, PRODUCTION_MAP + " not found; skipping");

    List<String> detours = new ArrayList<>();
    for (int seed : TIE_BREAK_SEEDS) {
      enableGuideAsProduction();
      int[] starts = new int[APRON_STARTS.length];
      int[] goals = new int[APRON_GOALS.length];
      for (int agent = 0; agent < starts.length; agent++) {
        starts[agent] = map.idOf(APRON_STARTS[agent]);
        goals[agent] = map.idOf(APRON_GOALS[agent]);
      }
      DistTable dist = DistTable.turnAware(map, goals);
      Footprint footprint = new Footprint(map);
      int[] config = starts.clone();
      int[][] carried = null;
      GuidedRoute route = null;
      for (int tick = 0; tick < APRON_TICKS; tick++) {
        FmsLocalGuide guide = FmsLocalGuide.builder(
            map,
            dist,
            goals,
            seed
        )
            .footprint(footprint)
            .loaded(APRON_LOADED)
            .build();
        if (carried != null) {
          guide.setGuidePaths(carried);
        }
        guide.construct(config, null, APRON_ORDER);
        carried = guide.currentGuidePaths();
        route = measure(map, carried[APRON_SUBJECT], goals[APRON_SUBJECT]);
        if (route.revisit() != null) {
          break;
        }
        int[] wanted = guide.guidedConfiguration();
        int[] next = config.clone();
        java.util.Set<Integer> taken = new HashSet<>();
        for (int agent : APRON_ORDER) {
          int target = wanted[agent] == CollisionTable.NO_VERTEX ? config[agent] : wanted[agent];
          if (taken.contains(target)) {
            target = config[agent];
          }
          taken.add(target);
          next[agent] = target;
        }
        config = next;
      }
      if (route.revisit() != null) {
        detours.add(String.format(
            "seed=%d %s@%s leaves and re-enters %s%n      compressed = %s",
            seed, APRON_SUBJECT_NAME, APRON_STARTS[APRON_SUBJECT], route.revisit(),
            route.compressed()));
      }
    }

    assertThat(detours)
        .as("%s stands loaded on %s with the apron ahead of it contested; stepping out to %s and"
            + " back costs exactly what waiting in place costs, so the guide may not spend two"
            + " moves to buy nothing. Captured from the fleet at 01:16:22 where it did.",
            APRON_SUBJECT_NAME, APRON_STARTS[APRON_SUBJECT], "0084")
        .isEmpty();
  }

  @Test
  void theTwoLoadedVehiclesCrossingAtTheJunctionAreNeverBothGivenItOnTheSameTick()
      throws Exception {
    Roadmap map = productionMap();
    assumeTrue(map != null, PRODUCTION_MAP + " not found; skipping");

    List<String> clashes = new ArrayList<>();
    for (int seed : TIE_BREAK_SEEDS) {
      enableGuideAsProduction();
      int[] starts = { map.idOf(JUNCTION_STARTS[0]), map.idOf(JUNCTION_STARTS[1]) };
      int[] goals = { map.idOf(JUNCTION_GOALS[0]), map.idOf(JUNCTION_GOALS[1]) };
      DistTable dist = DistTable.turnAware(map, goals);
      FmsLocalGuide guide = FmsLocalGuide.builder(
          map,
          dist,
          goals,
          seed
      )
          .footprint(new Footprint(map))
          .loaded(JUNCTION_LOADED)
          .build();
      guide.construct(starts, null, JUNCTION_ORDER);

      int[][] paths = guide.currentGuidePaths();
      for (int t = 0; t < paths[0].length && t < paths[1].length; t++) {
        if (paths[0][t] == CollisionTable.NO_VERTEX || paths[0][t] != paths[1][t]) {
          continue;
        }
        clashes.add(String.format(
            "seed=%d t=%d both stand on %s%n      a0 = %s%n      a1 = %s",
            seed, t, map.nameOf(paths[0][t]), asNames(map, paths[0]), asNames(map, paths[1])));
        break;
      }
    }

    assertThat(clashes)
        .as("%s plans first and enrols a path that occupies the junction %s; %s plans second and"
            + " must pay for standing there. Captured from the fleet at 09:28:33 where it did not,"
            + " so both loaded vehicles were handed the same cell on the same tick.",
            JUNCTION_STARTS[0], "0087", JUNCTION_STARTS[1])
        .isEmpty();
  }

  private static String asNames(Roadmap map, int[] path) {
    StringBuilder text = new StringBuilder("[");
    for (int t = 0; t < path.length; t++) {
      text.append(t == 0 ? "" : ", ")
          .append(path[t] == CollisionTable.NO_VERTEX ? "-" : map.nameOf(path[t]));
    }
    return text.append(']').toString();
  }

  private static void enableGuideAsProduction() {
    System.setProperty("fms.guide.on", "true");
    System.setProperty("fms.guide.deterministic", "false");
    System.setProperty("fms.guide.numRefine", "2");
    System.setProperty("fms.guide.collisionCost", DEFAULT_COLLISION_COST);
    System.setProperty("fms.guide.window", "20");
  }

  private static String queueLabelOf(int[] order) {
    return order[0] == LEADER ? "[leader,follower]" : "[follower,leader]";
  }

  private static int[] queueGoals(Roadmap map) {
    return new int[] { map.idOf(LEADER_GOAL), map.idOf(FOLLOWER_GOAL) };
  }

  private static int[] queueStarts(Roadmap map) {
    return new int[] { map.idOf(LANE_MOUTH), map.idOf(APRON_BEHIND) };
  }

  private static int[] queueNoses(Roadmap map) {
    return new int[] {
        map.noseAfter(Roadmap.DIR_NONE, map.idOf(OUTSIDE_THE_MOUTH), map.idOf(LANE_MOUTH)),
        map.noseAfter(Roadmap.DIR_NONE, map.idOf(APRON_FAR), map.idOf(APRON_BEHIND))
    };
  }

  private static FmsLocalGuide guideOverTheQueue(Roadmap map, int[] order, int seed) {
    int[] goals = queueGoals(map);
    FmsLocalGuide guide = FmsLocalGuide.builder(
        map,
        DistTable.turnAware(map, goals),
        goals,
        seed
    )
        .footprint(new Footprint(map))
        .loaded(new boolean[] { true, true })
        .build();
    guide.construct(queueStarts(map), queueNoses(map), order);
    return guide;
  }

  private static List<List<String>> driveTheQueue(Roadmap map, int[] order, int seed) {
    int[] goals = queueGoals(map);
    DistTable dist = DistTable.turnAware(map, goals);
    Footprint footprint = new Footprint(map);
    boolean[] loaded = { true, true };
    int[] config = queueStarts(map);
    int[] noses = queueNoses(map);
    List<String> leaderTrack = new ArrayList<>();
    List<String> followerTrack = new ArrayList<>();
    int[][] carried = null;

    for (int tick = 0; tick < QUEUE_TICKS; tick++) {
      leaderTrack.add(map.nameOf(config[LEADER]));
      followerTrack.add(map.nameOf(config[FOLLOWER]));
      FmsLocalGuide guide = FmsLocalGuide.builder(
          map,
          dist,
          goals,
          seed
      )
          .footprint(footprint)
          .loaded(loaded)
          .build();
      if (carried != null) {
        guide.setGuidePaths(carried);
      }
      guide.construct(config, noses, order);
      carried = guide.currentGuidePaths();
      config = advanceQueue(map, config, guide.guidedConfiguration(), order, noses);
    }
    return List.of(leaderTrack, followerTrack);
  }

  private static int[] advanceQueue(
      Roadmap map, int[] config, int[] wanted, int[] order, int[] noses) {
    int[] next = config.clone();
    Set<Integer> claimed = new HashSet<>();
    for (int agent : order) {
      int target = wanted[agent];
      if (target == CollisionTable.NO_VERTEX
          || claimed.contains(target)
          || swapsWithAnother(config, wanted, agent, target)) {
        target = config[agent];
      }
      claimed.add(target);
      next[agent] = target;
    }
    for (int agent = 0; agent < config.length; agent++) {
      noses[agent] = map.noseAfter(noses[agent], config[agent], next[agent]);
    }
    return next;
  }

  private static boolean swapsWithAnother(int[] config, int[] wanted, int agent, int target) {
    for (int other = 0; other < config.length; other++) {
      if (other != agent && config[other] == target && wanted[other] == config[agent]) {
        return true;
      }
    }
    return false;
  }

  private static void printHeadOn(Roadmap map, String header, FmsLocalGuide guide) {
    HeadOn outcome = analyse(map, guide);
    System.out.printf(
        "[FmsLocalGuide] %s window=%d %s%n",
        header, guide.window(),
        outcome.headOnStep() == NEVER
            ? "feasible (A enters the lane at t=" + orNever(outcome.firstStepAgentAinTheLane())
                + ", B clears it after t=" + orNever(outcome.lastStepAgentBinTheLane()) + ")"
            : "HEAD-ON at t=" + outcome.headOnStep());
    printRoute(AGENT_LABELS[WANTS_IN], outcome.wantsIn(), guide.collisionCostOf(WANTS_IN));
    printRoute(AGENT_LABELS[WANTS_OUT], outcome.wantsOut(), guide.collisionCostOf(WANTS_OUT));
  }

  private static void printRoute(String label, GuidedRoute route, float collisionCharge) {
    System.out.printf(
        "    %s waits=%d arrival=%s revisit=%s collisionCharge=%.1f%n"
            + "        full       = %s%n"
            + "        compressed = %s%n",
        label, route.waitSteps(), orNever(route.arrivalStep()), orNone(route.revisit()),
        collisionCharge, route.full(), route.compressed());
  }

  private static String orNone(String revisit) {
    return revisit == null ? "none" : revisit;
  }

  private static String orNever(int step) {
    return step == NEVER ? "never" : String.valueOf(step);
  }

  private static String labelOf(int[] order) {
    return order[0] == WANTS_IN ? "[A,B]" : "[B,A]";
  }

  private static void enableGuide(String collisionCost, String refineRounds) {
    System.setProperty("fms.guide.on", "true");
    System.setProperty("fms.guide.deterministic", "true");
    System.setProperty("fms.guide.numRefine", refineRounds);
    System.setProperty("fms.guide.collisionCost", collisionCost);
  }

  private static FmsLocalGuide guideOverTheHeadOn(Roadmap map, int[] order) {
    return guideOverTheHeadOn(map, order, SEED);
  }

  private static FmsLocalGuide guideOverTheHeadOn(Roadmap map, int[] order, int seed) {
    int[] goals = goalsOfTheHeadOn(map);
    int[] starts = { map.idOf(OUTSIDE_THE_MOUTH), map.idOf(ONE_BEFORE_THE_TIP) };
    DistTable dist = DistTable.turnAware(map, goals);
    FmsLocalGuide guide = FmsLocalGuide.builder(map, dist, goals, seed).build();
    guide.construct(starts, order);
    return guide;
  }

  private static int[] goalsOfTheHeadOn(Roadmap map) {
    return new int[] { map.idOf(LANE_TIP), map.idOf(MAINLINE_GOAL) };
  }

  private static int hopsBetween(Roadmap map, String from, String to) {
    return map.hopDistanceTo(map.idOf(to))[map.idOf(from)];
  }

  private static HeadOn analyse(Roadmap map, FmsLocalGuide guide) {
    int[] goals = goalsOfTheHeadOn(map);
    int[][] paths = guide.currentGuidePaths();
    GuidedRoute wantsIn = measure(map, paths[WANTS_IN], goals[WANTS_IN]);
    GuidedRoute wantsOut = measure(map, paths[WANTS_OUT], goals[WANTS_OUT]);
    int firstStepAgentAinTheLane = firstStepInsideTheLane(wantsIn.full());
    int lastStepAgentBinTheLane = lastStepInsideTheLane(wantsOut.full());
    int headOnStep = stepOfFirstLaneCellBefore(wantsIn.full(), lastStepAgentBinTheLane);
    return new HeadOn(
        wantsIn, wantsOut, firstStepAgentAinTheLane, lastStepAgentBinTheLane, headOnStep);
  }

  private static List<String> standOffOutsideTheMouthUntil(int windowLength) {
    int laneEntryStep = windowLength - LANE_CHAIN.size() - 2;
    List<String> path = new ArrayList<>();
    while (path.size() < laneEntryStep) {
      path.add(OUTSIDE_THE_MOUTH);
    }
    path.addAll(LANE_CHAIN.subList(1, LANE_CHAIN.size()));
    while (path.size() < windowLength) {
      path.add(LANE_TIP);
    }
    return path;
  }

  private static List<String> divertIntoTheMouthAndBackUntil(int windowLength) {
    int laneEntryStep = windowLength - LANE_CHAIN.size() - 2;
    List<String> path = new ArrayList<>();
    path.add(OUTSIDE_THE_MOUTH);
    while (path.size() + 1 < laneEntryStep) {
      path.add(LANE_MOUTH);
    }
    path.add(OUTSIDE_THE_MOUTH);
    path.addAll(LANE_CHAIN.subList(1, LANE_CHAIN.size()));
    while (path.size() < windowLength) {
      path.add(LANE_TIP);
    }
    return path;
  }

  private static int weightedCostOf(Roadmap map, List<String> path) {
    int cost = 0;
    int nose = Roadmap.DIR_NONE;
    for (int t = 1; t < path.size(); t++) {
      int from = map.idOf(path.get(t - 1));
      int to = map.idOf(path.get(t));
      int noseAfterTheStep = map.noseAfter(nose, from, to);
      cost += map.weightOf(to) + Roadmap.turnCostBetween(nose, noseAfterTheStep);
      nose = noseAfterTheStep;
    }
    return cost;
  }

  private static int movesIn(List<String> path) {
    int moves = 0;
    for (int t = 0; t + 1 < path.size(); t++) {
      if (!path.get(t).equals(path.get(t + 1))) {
        moves++;
      }
    }
    return moves;
  }

  private static List<String> compress(List<String> path) {
    List<String> compressed = new ArrayList<>();
    for (String point : path) {
      if (compressed.isEmpty() || !compressed.get(compressed.size() - 1).equals(point)) {
        compressed.add(point);
      }
    }
    return compressed;
  }

  private static int stepOfFirstLaneCellBefore(List<String> path, int lastBlockedStep) {
    for (int t = 0; t <= lastBlockedStep && t < path.size(); t++) {
      if (LANE_CELLS_WITH_NO_PASSING_PLACE.contains(path.get(t))) {
        return t;
      }
    }
    return NEVER;
  }

  private static int firstStepInsideTheLane(List<String> path) {
    for (int t = 0; t < path.size(); t++) {
      if (LANE_CELLS_WITH_NO_PASSING_PLACE.contains(path.get(t))) {
        return t;
      }
    }
    return NEVER;
  }

  private static int lastStepInsideTheLane(List<String> path) {
    int last = NEVER;
    for (int t = 0; t < path.size(); t++) {
      if (LANE_CELLS_WITH_NO_PASSING_PLACE.contains(path.get(t))) {
        last = t;
      }
    }
    return last;
  }

  private static GuidedRoute measure(Roadmap map, int[] path, int goal) {
    List<String> full = new ArrayList<>();
    for (int vertex : path) {
      full.add(vertex == CollisionTable.NO_VERTEX ? "--" : map.nameOf(vertex));
    }
    List<String> compressed = compress(full);
    int arrivalStep = NEVER;
    for (int t = 0; t < path.length; t++) {
      if (path[t] == goal) {
        arrivalStep = t;
        break;
      }
    }
    int lastCountedStep = arrivalStep == NEVER ? path.length - 1 : arrivalStep;
    int waitSteps = 0;
    for (int t = 0; t + 1 <= lastCountedStep; t++) {
      if (path[t] == path[t + 1]) {
        waitSteps++;
      }
    }
    return new GuidedRoute(full, compressed, waitSteps, arrivalStep, firstRepeat(compressed));
  }

  private static String firstRepeat(List<String> compressed) {
    for (int i = 0; i < compressed.size(); i++) {
      for (int j = i + 1; j < compressed.size(); j++) {
        if (compressed.get(i).equals(compressed.get(j))) {
          return compressed.get(i);
        }
      }
    }
    return null;
  }

  private static Roadmap productionMap() throws Exception {
    if (!productionMapLookedUp) {
      productionMapLookedUp = true;
      File file = RoadmapXmlLoader.findMap(PRODUCTION_MAP);
      cachedProductionMap = file == null ? null : new Roadmap(RoadmapXmlLoader.load(file));
    }
    return cachedProductionMap;
  }

  private static int waitStepsBeforeArrival(int[] path, int goal) {
    int arrival = arrivalTime(path, goal);
    int waits = 0;
    for (int t = 0; t + 1 < arrival; t++) {
      if (path[t] == path[t + 1]) {
        waits++;
      }
    }
    return waits;
  }

  private static int arrivalTime(int[] path, int goal) {
    for (int t = 0; t < path.length; t++) {
      if (path[t] == goal) {
        return t;
      }
    }
    return path.length - 1;
  }

  private static int firstTimeAt(int[] path, int vertex) {
    for (int t = 0; t < path.length; t++) {
      if (path[t] == vertex) {
        return t;
      }
    }
    return path.length;
  }

  private static String render(Roadmap roadmap, int[] path, int goal) {
    StringBuilder rendered = new StringBuilder("[");
    int arrival = arrivalTime(path, goal);
    for (int t = 0; t <= arrival; t++) {
      rendered.append(t == 0 ? "" : " ").append(roadmap.nameOf(path[t]));
    }
    return rendered.append("]").toString();
  }

  private static String cell(int index) {
    return "c" + index;
  }

  private static Roadmap corridorWithSidePocket() {
    List<Roadmap.Point> points = new ArrayList<>();
    for (int i = 0; i < CORRIDOR_LENGTH; i++) {
      points.add(new Roadmap.Point(cell(i), i, 0, 0));
    }
    points.add(new Roadmap.Point(POCKET, 2, 1, 0));

    List<Roadmap.Path> paths = new ArrayList<>();
    for (int i = 0; i + 1 < CORRIDOR_LENGTH; i++) {
      paths.add(new Roadmap.Path(
          cell(i) + "->" + cell(i + 1), cell(i), cell(i + 1), 1, true));
    }
    paths.add(new Roadmap.Path(cell(2) + "->" + POCKET, cell(2), POCKET, 1, true));
    return new Roadmap(new Roadmap.Model(points, paths));
  }

  private record GuidedRoute(
      List<String> full, List<String> compressed, int waitSteps, int arrivalStep, String revisit) {
  }

  private record HeadOn(
      GuidedRoute wantsIn,
      GuidedRoute wantsOut,
      int firstStepAgentAinTheLane,
      int lastStepAgentBinTheLane,
      int headOnStep) {
  }
  private static final String MOUTH_OUTSIDE = "0085";
  private static final String MOUTH_APPROACH = "0084";
  private static final String MOUTH_COLUMN_TOP = "0174";
  private static final String MOUTH_OUTBOUND_START = "0164";
  private static final String MOUTH_GOAL_TURNING_EAST = "0091";
  private static final String MOUTH_GOAL_STRAIGHT_SOUTH = "0004";
  private static final String MOUTH_HEADON_WAITING = "0086";
  private static final String MOUTH_HEADON_OUTBOUND = "0104";
  @Test
  void reportsTheMouthHeadOnAcrossStartsAndGoals() throws Exception {
    Roadmap map = productionMap();
    assumeTrue(map != null, PRODUCTION_MAP + " not found; skipping");

    for (String waitingStart : new String[] { MOUTH_HEADON_WAITING, MOUTH_OUTSIDE }) {
      for (String outboundStart : new String[] { MOUTH_HEADON_OUTBOUND, MOUTH_OUTBOUND_START }) {
        for (String outboundGoal
            : new String[] { MOUTH_GOAL_TURNING_EAST, MOUTH_GOAL_STRAIGHT_SOUTH }) {
          enableGuideAsProduction();
          int[] goals = { map.idOf(MOUTH_COLUMN_TOP), map.idOf(outboundGoal) };
          DistTable dist = DistTable.turnAware(map, goals);
          Footprint footprint = new Footprint(map);
          boolean[] loaded = { true, true };
          int[] config = { map.idOf(waitingStart), map.idOf(outboundStart) };
          int[] noses = {
              map.noseAfter(Roadmap.DIR_NONE, map.idOf(MOUTH_APPROACH), config[WAITING]),
              map.noseAfter(Roadmap.DIR_NONE, map.idOf(MOUTH_COLUMN_TOP), config[OUTBOUND])
          };
          FmsLocalGuide guide =
              FmsLocalGuide.builder(map, dist, goals, 0).footprint(footprint).loaded(loaded).build();
          guide.construct(config, noses, WAITING_FIRST);
          int[][] paths = guide.currentGuidePaths();
          System.out.printf("[FmsLocalGuide/headon] A@%s B@%s->%s%n    A = %s%n    B = %s%n",
              waitingStart, outboundStart, outboundGoal,
              measure(map, paths[WAITING], goals[WAITING]).full(),
              measure(map, paths[OUTBOUND], goals[OUTBOUND]).full());
        }
      }
    }
  }

  // --- the guide obeys the same kinematics PIBT and the constraint tree do ------

  private static int directionChangesWithoutBraking(Roadmap map, int[] path) {
    int offences = 0;
    int motion = Roadmap.DIR_NONE;
    for (int t = 0; t + 1 < path.length; t++) {
      if (path[t] < 0 || path[t + 1] < 0) {
        break;
      }
      int next = map.motionAfter(path[t], path[t + 1]);
      if (motion != Roadmap.DIR_NONE && next != Roadmap.DIR_NONE && next != motion) {
        offences++;
      }
      motion = next;
    }
    return offences;
  }

  @Test
  void theGuidePathNeverTurnsWithoutStandingStillFirst() {
    Roadmap map = GridFixture.grid(6, 6);
    System.setProperty("fms.guide.on", "true");
    System.setProperty("fms.guide.window", "12");
    System.setProperty("fms.guide.numRefine", "1");
    System.setProperty("fms.guide.deterministic", "true");

    int[] starts = {GridFixture.id(map, 0, 0), GridFixture.id(map, 5, 5)};
    int[] goals = {GridFixture.id(map, 4, 3), GridFixture.id(map, 1, 2)};
    FmsLocalGuide guide = FmsLocalGuide.builder(
        map,
        DistTable.turnAware(map, goals),
        goals,
        SEED
    )
        .build();
    guide.construct(
        starts,
        new int[] {Roadmap.DIR_POS_X, Roadmap.DIR_NEG_X},
        new int[] {Roadmap.DIR_POS_X, Roadmap.DIR_NEG_X},
        new int[] {0, 1},
        0);

    for (int agent = 0; agent < goals.length; agent++) {
      assertThat(directionChangesWithoutBraking(map, guide.currentGuidePaths()[agent]))
          .as("agent %d is offered a corner it cannot drive: PIBT would refuse the step and the"
              + " guide would be leading it nowhere", agent)
          .isZero();
    }
  }

  @Test
  void aGuidedAgentThatStartsRollingMayNotTurnOnItsFirstStep() {
    Roadmap map = GridFixture.grid(4, 4);
    System.setProperty("fms.guide.on", "true");
    System.setProperty("fms.guide.window", "8");
    System.setProperty("fms.guide.numRefine", "1");
    System.setProperty("fms.guide.deterministic", "true");

    int[] starts = {GridFixture.id(map, 1, 1)};
    int[] goals = {GridFixture.id(map, 3, 1)};
    FmsLocalGuide guide = FmsLocalGuide.builder(
        map,
        DistTable.turnAware(map, goals),
        goals,
        SEED
    )
        .build();
    guide.construct(
        starts,
        new int[] {Roadmap.DIR_POS_X},
        new int[] {Roadmap.DIR_POS_X},
        new int[] {0},
        0);
    int[] path = guide.currentGuidePaths()[0];

    assertThat(map.motionAfter(path[0], path[1]))
        .as("the goal is north but the vehicle is rolling east, so the only legal first steps"
            + " are carrying on east or braking; turning north now is not one of them")
        .isIn(Roadmap.DIR_POS_X, Roadmap.DIR_NONE);
  }
}
