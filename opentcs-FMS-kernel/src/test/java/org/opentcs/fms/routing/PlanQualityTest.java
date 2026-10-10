package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Deque;
import java.util.List;
import java.util.Locale;
import java.util.Random;
import java.util.Set;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.pibt.Footprint;

class PlanQualityTest {

  private static final String MAP_FILE = "v7-vda5050-test.xml";
  private static final long DEADLINE_MS = 120_000L;
  private static final int LOOP_LIMIT = 8_000;
  private static final int WINDOW = 10;
  private static final double UPPER_MAINLINE = 1000;
  private static final double LOWER_MAINLINE = 0;
  private static final int[] WINDOWED_FLEET_SIZES = { 10, 20, 30 };
  private static final int[] SPRINT_FLEET_SIZES = { 4, 8, 16 };
  private static final int[] SEEDS = { 1, 2, 3 };

  private static final double MIN_PROGRESS_RATE = 0.70;
  private static final double MAX_WOBBLE_RATE = 0.05;
  private static final double MAX_STALLED_SHARE = 0.20;
  private static final double MAX_WAIT_RATE = 0.20;
  private static final double MAX_SUBOPTIMALITY = 1.75;

  private static final List<String> CONVOY_POCKETS = List.of("1006", "1005", "1002", "1004", "1001", "1003");
  private static final List<String> CONVOY_GOALS = List.of("0104", "0105", "0106", "0107", "0108", "0109");
  private static final String PICKUP_LANE_MOUTH = "0086";
  private static final List<String> RACK_COLUMN_STARTS = List.of("0110", "0084", "0086");
  private static final List<String> RACK_COLUMN_GOALS = List.of("3043", "3044", "3045");
  private static final String RACK_COLUMN_MOUTH = "3042";
  private static final long PRODUCTION_DEADLINE_MS = 1_000L;
  private static final int RACK_COLUMN_SEEDS = 8;
  private static final String GUIDE_PROPERTY = "fms.guide.on";
  private static final String REFINE_PROPERTY = "fms.guide.numRefine";
  private static final String CLEAR_GOAL_PROPERTY = "fms.guide.clearGoalFirst";
  private static final double CELL_PITCH_MM = 1000;
  private static final double SHIPPED_CLEARANCE_MM = 50;
  private static final List<String> PICKUP_LANE = List.of("0104", "0114", "0124", "0134", "0144", "0154", "0164",
      "0174");

  private static final String XUONG_MAP_FILE = "v7-map-xuong.xml";
  private static final List<String> XUONG_CARGO_SLOTS = List.of("0510", "0511", "0515", "0516", "0520", "0521");

  private static Roadmap map;
  private static List<Quality> windowedPlans;
  private static List<Quality> sprintPlans;

  @BeforeAll
  static void planTheCorpusAndScoreIt() throws Exception {
    File file = RoadmapXmlLoader.findMap(MAP_FILE);
    assumeTrue(file != null, MAP_FILE + " not found; skipping");
    map = new Roadmap(RoadmapXmlLoader.load(file));

    int[] core = stronglyConnectedCore(map);
    assumeTrue(core.length > 2 * maxOf(WINDOWED_FLEET_SIZES),
        "the strongly connected core holds too few points for the largest fleet");

    windowedPlans = new ArrayList<>();
    for (int fleet : WINDOWED_FLEET_SIZES) {
      for (int seed : SEEDS) {
        windowedPlans.add(planAndScore(instanceOn(core, fleet, seed), WINDOW));
      }
    }
    sprintPlans = new ArrayList<>();
    for (int fleet : SPRINT_FLEET_SIZES) {
      for (int seed : SEEDS) {
        sprintPlans.add(planAndScore(instanceOn(core, fleet, seed), SolverParams.WINDOW_FULL));
      }
    }
    System.out.print(report());
  }

  @Test
  void everyPlanInTheCorpusIsCollisionFreeAndWalksRealEdges() {
    for (Quality quality : allPlans()) {
      assertThat(quality.violations()).as("%s", label(quality)).isEmpty();
    }
  }

  @Test
  void theFeasibilityChecksFireWhenAPlanIsCorrupted() {
    Quality clean = windowedPlans.get(0);
    int[] parked = clean.plan().get(clean.makespan());
    int elsewhere = clean.instance().goals()[0];

    assertThat(map.distance(parked[0], elsewhere))
        .as("the corruption below is only a teleport if there is no single edge to ride")
        .isGreaterThan(1);
    assertThat(violationsAfterMoving(clean, 0, parked[1]))
        .as("a report of zero violations is worth reading only if two vehicles sharing one point"
            + " would have been reported")
        .isNotEmpty();
    assertThat(violationsAfterMoving(clean, 0, elsewhere))
        .as("and only if a jump with no edge under it would have been reported")
        .isNotEmpty();
  }

  @Test
  void everyWindowedPlanFillsExactlyTheWindowItWasGiven() {
    for (Quality quality : windowedPlans) {
      assertThat(quality.reachedGoals())
          .as("%s: goals are drawn from the whole map, so no fleet reaches them inside %d steps;"
              + " a plan that claims it did is measuring something other than the window",
              label(quality), WINDOW)
          .isFalse();
      assertThat(quality.makespan())
          .as("%s: stopping short throws away horizon the fleet paid for, and overrunning hands"
              + " the caller steps nobody validated", label(quality))
          .isEqualTo(WINDOW);
    }
  }

  @Test
  void everySprintPlanActuallyArrives() {
    for (Quality quality : sprintPlans) {
      assertThat(quality.reachedGoals())
          .as("%s runs unwindowed, so a plan that stops short is a failure, not a truncation",
              label(quality))
          .isTrue();
    }
  }

  @Test
  void aVehicleRidesThePlainPointRatherThanCutThroughAnEquallyCloseLocationPoint() {
    Instance lone = new Instance(
        1, 0, new int[] { map.idOf("3090") }, new int[] { map.idOf("3108") });

    assertThat(map.weightOf(map.idOf("3099")))
        .as("3099 and 3110 are both one hop past 3100 and one hop short of 3109, so hop count"
            + " cannot separate the two routes; only the surcharge on 3099's location link can")
        .isGreaterThan(map.weightOf(map.idOf("3110")));

    Lacam2Solver.Result result = solve(lone, SolverParams.WINDOW_FULL);

    assertThat(routeOf(result, 0))
        .as("the detour through location_3099 costs the same in hops and more in weight, so a"
            + " plan that takes it is routing a vehicle across a cargo slot for nothing")
        .containsExactly("3090", "3100", "3110", "3109", "3108");
  }

  @Test
  void theRetreatFrom0515BacksStraightUpItsOwnColumn() throws Exception {
    File file = RoadmapXmlLoader.findMap(XUONG_MAP_FILE);
    assumeTrue(file != null, XUONG_MAP_FILE + " not found; skipping");
    Roadmap.Model shipped = RoadmapXmlLoader.load(file);
    Roadmap stocked = new Roadmap(new Roadmap.Model(
        shipped.points(), shipped.paths(), Set.of(), Set.of("0515", "0510")));
    Instance lone = loneTrip(stocked, "0515", "0505");
    List<String> straightUp = List.of("0515", "0510", "0505");

    List<List<String>> runs = new ArrayList<>();
    for (int run = 0; run < 5; run++) {
      runs.add(routeOf(stocked, solve(stocked, lone, SolverParams.WINDOW_FULL), 0));
    }

    assertThat(runs)
        .as("0510 is the only cell between the vehicle and 0505 and it carries the drop-off"
            + " weight, but the way round it leaves the column at 0514, climbs column C and comes"
            + " back down through 0001 -- eight hops and four turns against two hops and none, so"
            + " no setting of the drop-off weight below the turn budget can buy that detour")
        .containsOnly(straightUp);
  }

  @Test
  void aCargoSurchargeOnBothXuongColumnsPricesPibtOffTheShorterOneWithoutMovingThePlan()
      throws Exception {
    File file = RoadmapXmlLoader.findMap(XUONG_MAP_FILE);
    assumeTrue(file != null, XUONG_MAP_FILE + " not found; skipping");
    Roadmap.Model shipped = RoadmapXmlLoader.load(file);
    Roadmap bare = new Roadmap(shipped);
    Roadmap stocked = new Roadmap(
        new Roadmap.Model(shipped.points(), shipped.paths(), Set.copyOf(XUONG_CARGO_SLOTS)));
    int start = stocked.idOf("0521");
    int goal = stocked.idOf("0506");
    int surcharged = Roadmap.cargoSlotWeight(stocked.size());
    List<String> upTheColumn = List.of("0521", "0516", "0511", "0506");

    assertThat(shipped.cargoSlotPoints())
        .as("%s links only chargers, so its surcharge is inert as shipped and the six slots"
            + " below are this test bolting locations on rather than the file carrying them",
            XUONG_MAP_FILE)
        .isEmpty();
    assertThat(surcharged)
        .as("the surcharge is big-M by construction -- one per point in the map, plus one -- so"
            + " in any model that adds it up, crossing one slot fewer outprices every detour"
            + " this map is able to offer")
        .isGreaterThan(stocked.size());
    for (int exit : stocked.neighborsOf(start)) {
      assertThat(stocked.weightOf(exit))
          .as("%s: both ways out of 0521 are among the six, so no route leaves the block without"
              + " paying at least once and the only question is how many more it pays",
              stocked.nameOf(exit))
          .isEqualTo(surcharged);
    }

    assertThat(routeOf(bare, solve(bare, loneTrip(bare, "0521", "0506"), SolverParams.WINDOW_FULL),
        0))
        .as("before the locations go on, the pair sits three hops apart up one column and the"
            + " vehicle simply climbs it")
        .containsExactlyElementsOf(upTheColumn);

    assertThat(stocked.distance(start, goal))
        .as("with them on, the weighted table reprices the trip as the long way round: leaving"
            + " the block through 0520 charges one big-M, while climbing through 0516 and 0511"
            + " charges two")
        .isGreaterThan(bare.distance(bare.idOf("0521"), bare.idOf("0506")));
    assertThat(costThrough(stocked, stocked.idOf("0520"), goal))
        .as("so PIBT, which ranks a candidate by weightOf(v) + dist(v, goal), wants the long way"
            + " round: both first steps are surcharged, and only the one out of the block avoids"
            + " a second charge")
        .isLessThan(costThrough(stocked, stocked.idOf("0516"), goal));

    assertThat(routeOf(stocked, solve(stocked, loneTrip(stocked, "0521", "0506"),
        SolverParams.WINDOW_FULL), 0))
        .as("and the plan climbs the column regardless. The surcharge reaches PIBT and the"
            + " priority order through DistTable.weighted, but LaCAM scores a high-level node"
            + " with hValue over DistTable.hops and an edge cost of one tick per off-goal agent,"
            + " so the node it keeps is whichever sits fewest hops from the goal. The surcharge"
            + " can only settle ties between routes of equal length; it cannot buy the eight"
            + " extra hops that going round would cost")
        .containsExactlyElementsOf(upTheColumn);
    assertThat(routeOf(stocked, solve(stocked, loneTrip(stocked, "0521", "0506"), 1), 0))
        .as("and shrinking the horizon to a single tick does not hand the decision back: the"
            + " high-level ranking is hop-based at every window, so even one step ahead the"
            + " vehicle takes the point PIBT ranked second")
        .containsExactly("0521", "0516");

    assertThat(surchargePaidOn(stocked, upTheColumn))
        .as("the plan therefore drives over two cargo slots on a map where one was reachable,"
            + " which is the cost the surcharge exists to avoid and here does not")
        .isEqualTo(2 * (surcharged - 1));
    assertThat(turnsIn(stocked, upTheColumn))
        .as("what it does not cost is rotation -- the column is straight -- so the detour the"
            + " surcharge asked for would have been paid for twice over on the floor")
        .isZero();
    assertThat(stocked.hopDistanceTo(goal)[start])
        .as("and no edge moved under any of this: the table the search actually ranks on reads"
            + " the same three hops it read before the locations went on")
        .isEqualTo(bare.hopDistanceTo(bare.idOf("0506"))[bare.idOf("0521")])
        .isEqualTo(upTheColumn.size() - 1);
  }

  @Test
  void sixPocketedVehiclesLeaveForOneCorridorAndTheDumpShowsWhoBacksIntoADeadEnd() {
    StringBuilder report = new StringBuilder();
    for (int window : new int[] { WINDOW, SolverParams.WINDOW_FULL }) {
      for (int seed : SEEDS) {
        report.append(convoyPlanDump("shuffled goals seed=" + seed, shuffledGoals(seed), window));
      }
    }
    System.out.print(report);

    assertThat(report.length()).isPositive();
  }

  @Test
  void theSixPocketConvoyPlannedInOneShotIsScoredForOscillation() {
    StringBuilder report = new StringBuilder(String.format(Locale.ROOT,
        "%n[one-shot convoy] starts=%s goals=%s%n%-10s %5s %6s %8s %6s %7s %9s %8s %7s%n",
        CONVOY_POCKETS, CONVOY_GOALS,
        "pairing", "seed", "ticks", "arrived", "yield", "wobble", "reversals", "revisits",
        "loops"));
    int wobbles = 0;
    int revisits = 0;
    for (boolean crossed : new boolean[] { false, true }) {
      List<String> goals = new ArrayList<>(CONVOY_GOALS);
      if (crossed) {
        Collections.reverse(goals);
      }
      for (int seed : SEEDS) {
        Instance convoy = new Instance(
            CONVOY_POCKETS.size(), seed, idsOf(CONVOY_POCKETS), idsOf(goals));
        Lacam2Solver.Result result = solve(convoy, SolverParams.WINDOW_FULL);
        int[][] paths = transpose(result.solution(), convoy.agents());
        int[] backsteps = backstepKinds(paths);
        int repeated = revisitedPoints(paths);
        wobbles += backsteps[1];
        revisits += repeated;

        report.append(String.format(Locale.ROOT,
            "%-10s %5d %6d %8b %6d %7d %9d %8d %7d%n",
            crossed ? "reversed" : "in-order", seed, result.solution().size() - 1,
            result.reachedGoals(), backsteps[0], backsteps[1],
            reversals(result.solution(), convoy.agents()), repeated, result.loopCount()));
        for (int agent = 0; agent < convoy.agents(); agent++) {
          report.append(String.format(Locale.ROOT, "  %s -> %-6s %s%n",
              CONVOY_POCKETS.get(agent), goals.get(agent), routeOf(result, agent)));
        }

        assertThat(result.reachedGoals())
            .as("%s seed=%d: the convoy is planned unwindowed, so a plan that stops short of the"
                + " goals is a failure rather than a truncation", goals, seed)
            .isTrue();
        assertThat(GridFixture.findCollisions(paths))
            .as("%s seed=%d: an oscillation count is only worth reading off a feasible plan",
                goals, seed)
            .isEmpty();
      }
    }
    System.out.print(report);

    assertThat(wobbles)
        .as("a wobble is a vehicle stepping off a point and back while no other vehicle used the"
            + " point it vacated; one-shot search has the whole horizon in view here and never"
            + " plans one, under either pairing")
        .isZero();
    assertThat(revisits)
        .as("no vehicle stands on the same point twice either, so the plans carry no oscillation"
            + " of any shape, not merely none of the one-step-back shape")
        .isZero();
    assertThat(backEdgesAlongTheCorridor())
        .as("and the corridor is not what stops them: every point 0075..0090 carries an edge"
            + " back to the point behind it, so backing up was available at every step of every"
            + " route and the search declined it")
        .isEqualTo(16);
  }

  private static int revisitedPoints(int[][] paths) {
    int repeats = 0;
    for (int[] path : paths) {
      Set<Integer> standingsSoFar = new java.util.HashSet<>();
      int previous = -1;
      for (int point : path) {
        if (point == previous) {
          continue;
        }
        if (!standingsSoFar.add(point)) {
          repeats++;
        }
        previous = point;
      }
    }
    return repeats;
  }

  private static int backEdgesAlongTheCorridor() {
    int backwards = 0;
    for (int i = 75; i < 91; i++) {
      int here = map.idOf(String.format(Locale.ROOT, "%04d", i));
      int behind = map.idOf(String.format(Locale.ROOT, "%04d", i - 1));
      for (int neighbor : map.neighborsOf(here)) {
        if (neighbor == behind) {
          backwards++;
        }
      }
    }
    return backwards;
  }

  private static String convoyPlanDump(String pairing, List<String> goals, int window) {
    int agents = CONVOY_POCKETS.size();
    Instance convoy = new Instance(agents, 1, idsOf(CONVOY_POCKETS), idsOf(goals));
    Lacam2Solver.Result result = solve(convoy, window);
    List<int[]> plan = result.solution();

    StringBuilder out = new StringBuilder(String.format(Locale.ROOT,
        "%n[convoy] %s | window=%s | ticks=%d loops=%d nodes=%d optimal=%b reachedGoals=%b%n",
        pairing, window == SolverParams.WINDOW_FULL ? "full" : String.valueOf(window),
        plan.size() - 1, result.loopCount(), result.nodesGenerated(), result.optimal(),
        result.reachedGoals()));

    out.append(row("start", CONVOY_POCKETS));
    out.append(row("goal", goals));
    List<String> distances = new ArrayList<>(agents);
    for (int agent = 0; agent < agents; agent++) {
      distances.add(
          String.valueOf(map.distance(convoy.starts()[agent], convoy.goals()[agent])));
    }
    out.append(row("dist", distances));

    for (int tick = 0; tick < plan.size(); tick++) {
      List<String> standing = new ArrayList<>(agents);
      for (int agent = 0; agent < agents; agent++) {
        standing.add(map.nameOf(plan.get(tick)[agent]));
      }
      out.append(row("t" + tick, standing));
    }

    for (int agent = 0; agent < agents; agent++) {
      out.append(String.format(Locale.ROOT, "  %s -> %-6s %s%n",
          CONVOY_POCKETS.get(agent), goals.get(agent), routeOf(result, agent)));
    }
    out.append(String.format(Locale.ROOT, "  deadEndDives=%d reversals=%d%n",
        deadEndDives(plan, convoy.goals(), agents), reversals(plan, agents)));
    return out.toString();
  }

  private static String row(String label, List<String> cells) {
    StringBuilder line = new StringBuilder(String.format(Locale.ROOT, "%-7s", label));
    for (String cell : cells) {
      line.append(String.format(Locale.ROOT, " %-6s", cell));
    }
    return line.append(System.lineSeparator()).toString();
  }

  private static int[] idsOf(List<String> points) {
    int[] ids = new int[points.size()];
    for (int i = 0; i < ids.length; i++) {
      ids[i] = map.idOf(points.get(i));
    }
    return ids;
  }

  private static List<String> shuffledGoals(int seed) {
    List<String> shuffled = new ArrayList<>(CONVOY_GOALS);
    Collections.shuffle(shuffled, new Random(seed));
    return shuffled;
  }

  private static int deadEndDives(List<int[]> plan, int[] goals, int agents) {
    int dives = 0;
    for (int agent = 0; agent < agents; agent++) {
      for (int tick = 1; tick < plan.size(); tick++) {
        int arrivedAt = plan.get(tick)[agent];
        if (arrivedAt != plan.get(tick - 1)[agent]
            && arrivedAt != goals[agent]
            && map.neighborsOf(arrivedAt).length == 1) {
          dives++;
        }
      }
    }
    return dives;
  }

  private static int reversals(List<int[]> plan, int agents) {
    int turnarounds = 0;
    for (int agent = 0; agent < agents; agent++) {
      int justLeft = -1;
      int standing = plan.get(0)[agent];
      for (int tick = 1; tick < plan.size(); tick++) {
        int arrivedAt = plan.get(tick)[agent];
        if (arrivedAt == standing) {
          continue;
        }
        if (arrivedAt == justLeft) {
          turnarounds++;
        }
        justLeft = standing;
        standing = arrivedAt;
      }
    }
    return turnarounds;
  }

  @Test
  void planning0025To1000ThreeTimesRepeatsItselfAndKeepsTheHeuristicHonest() {
    int start = map.idOf("0025");
    int goal = map.idOf("1000");
    Instance lone = new Instance(1, 0, new int[] { start }, new int[] { goal });

    List<List<String>> runs = new ArrayList<>();
    for (int run = 0; run < 3; run++) {
      runs.add(routeOf(solve(lone, SolverParams.WINDOW_FULL), 0));
    }

    assertThat(runs)
        .as("the loop budget rather than the wall clock ends the search and the tie-break RNG is"
            + " seeded, so repeated planning of one instance cannot drift")
        .containsOnly(List.of("0025", "0024", "0071", "1000"));

    assertThat(map.weightOf(goal))
        .as("point 1000 carries a location link, but it is Charging-1000, and cargoSlotPoints()"
            + " skips chargers, so the goal is not surcharged")
        .isEqualTo(1);
    assertThat(map.distance(start, goal))
        .as("no point on this route is a cargo slot, so the heuristic and the hop count agree"
            + " and the bound the search prunes against is honest")
        .isEqualTo(hopsTo(goal)[start]);
    assertThat(map.distance(map.idOf("3074"), map.idOf("0011")))
        .as("one cargo slot on the 3074 route lifts the same heuristic above the hop count it is"
            + " supposed to bound, which is where the pruning bound starts over-estimating")
        .isGreaterThan(hopsTo(map.idOf("0011"))[map.idOf("3074")]);
  }

  @Test
  void planning0202To0052EightTimesRepeatsItselfThoughTheTiedRivalCostsAnExtraTurn() {
    int start = map.idOf("0202");
    int goal = map.idOf("0052");
    Instance lone = new Instance(1, 0, new int[] { start }, new int[] { goal });
    List<String> overTheTop = List.of("0202", "0045", "0050", "0051", "0052");
    List<String> alongTheBottom = List.of("0202", "0045", "0044", "0043", "0052");

    List<List<String>> runs = new ArrayList<>();
    for (int run = 0; run < 8; run++) {
      runs.add(routeOf(solve(lone, SolverParams.WINDOW_FULL), 0));
    }

    assertThat(runs)
        .as("nothing in the search reads the clock, so eight replans of one instance land on one"
            + " route; seed 3 lands on the rival, which is how little separates them")
        .containsOnly(overTheTop);

    assertThat(map.distance(start, goal))
        .as("neither way round the block touches a cargo slot, so here the heuristic is exactly"
            + " the hop count and the pruning bound is honest")
        .isEqualTo(hopsTo(goal)[start]);
    assertThat(map.distance(map.idOf("0050"), goal))
        .as("both ways round are the same length, so no term in the cost model prefers either")
        .isEqualTo(map.distance(map.idOf("0044"), goal));
    assertThat(turnsIn(overTheTop))
        .as("yet they differ on the floor: the rival spends a second ninety-degree rotation, and"
            + " a model built from vertex weights alone has nowhere to charge it")
        .isLessThan(turnsIn(alongTheBottom));
  }

  @Test
  void nothingSeparatesTheTwoMainlinesSoAVehicleStaysOnWhicheverItStartedOn() {
    assertThat(cargoSlotsOn(UPPER_MAINLINE))
        .as("neither mainline carries a location link, so the surcharge cannot favour one")
        .isEqualTo(cargoSlotsOn(LOWER_MAINLINE))
        .isZero();

    int[][] trips = {
        { map.idOf("0047"), map.idOf("0004") },
        { map.idOf("0004"), map.idOf("0047") },
        { map.idOf("0048"), map.idOf("0091") },
        { map.idOf("0091"), map.idOf("0048") } };
    for (int[] trip : trips) {
      double lane = map.yOf(trip[0]);
      List<String> route = routeOf(
          solve(new Instance(1, 0, new int[] { trip[0] }, new int[] { trip[1] }),
              SolverParams.WINDOW_FULL),
          0);

      assertThat(route)
          .as("%s -> %s: switching mainline costs two hops and buys nothing, so the planner has"
              + " no eastbound or westbound preference to express",
              map.nameOf(trip[0]), map.nameOf(trip[1]))
          .allMatch(point -> map.yOf(map.idOf(point)) == lane);
      assertThat(turnsIn(route))
          .as("%s -> %s: a lane change would show up here as two rotations",
              map.nameOf(trip[0]), map.nameOf(trip[1]))
          .isZero();
    }
  }

  @Test
  void headOnTrafficMakesPibtBorrowTheOtherMainlineAsAPassingBay() {
    List<Integer> lane = pointsOn(LOWER_MAINLINE);
    int fleet = 5;
    int[] starts = new int[fleet * 2];
    int[] goals = new int[fleet * 2];
    for (int i = 0; i < fleet; i++) {
      starts[i] = lane.get(8 + i);
      goals[i] = lane.get(lane.size() - 9 - i);
      starts[fleet + i] = lane.get(lane.size() - 8 - i);
      goals[fleet + i] = lane.get(9 + i);
    }

    Lacam2Solver.Result result = solve(
        new Instance(fleet * 2, 0, starts, goals), SolverParams.WINDOW_FULL);
    int[][] paths = transpose(result.solution(), fleet * 2);

    assertThat(result.reachedGoals())
        .as("five vehicles each way on one lane still has to resolve")
        .isTrue();
    assertThat(GridFixture.findCollisions(paths)).isEmpty();
    assertThat(ticksOn(paths, UPPER_MAINLINE))
        .as("every vehicle starts and ends on the lower mainline, so any time spent on the upper"
            + " one is the solver choosing to route around oncoming traffic; a lone vehicle"
            + " never does this, which is why single-agent probes cannot see lane usage at all")
        .isPositive();
  }

  private static String planDump(String title, Instance instance, Lacam2Solver.Result result) {
    List<int[]> plan = result.solution();
    StringBuilder out = new StringBuilder(String.format(Locale.ROOT,
        "%n[pair] %s | ticks=%d loops=%d nodes=%d optimal=%b reachedGoals=%b ms=%d%n",
        title, plan.size() - 1, result.loopCount(), result.nodesGenerated(), result.optimal(),
        result.reachedGoals(), result.runtimeMs()));
    out.append(row("start", namesOf(instance.starts())));
    out.append(row("goal", namesOf(instance.goals())));
    for (int tick = 0; tick < plan.size(); tick++) {
      out.append(row("t" + tick, namesOf(plan.get(tick))));
    }
    for (int agent = 0; agent < instance.agents(); agent++) {
      out.append(String.format(Locale.ROOT, "  %s -> %-6s %s%n",
          map.nameOf(instance.starts()[agent]), map.nameOf(instance.goals()[agent]),
          routeOf(result, agent)));
    }
    return out.toString();
  }

  private static List<String> namesOf(int[] vertices) {
    List<String> names = new ArrayList<>(vertices.length);
    for (int vertex : vertices) {
      names.add(map.nameOf(vertex));
    }
    return names;
  }

  @Test
  void theVehicleNearestTheRackColumnTakesTheShallowCellAndIsWalkedRoundTheBlockForIt()
      throws Exception {
    int[] starts = idsOf(RACK_COLUMN_STARTS);
    int[] goals = idsOf(RACK_COLUMN_GOALS);
    int[] noses = { Roadmap.DIR_NEG_Y, Roadmap.DIR_NEG_X, Roadmap.DIR_NEG_X };
    boolean[] loaded = { true, true, true };
    Instance intoTheColumn = new Instance(3, 0, starts, goals);
    Footprint footprint = new Footprint(map);

    for (int i = 1; i < goals.length; i++) {
      assertThat(map.xOf(goals[i]))
          .as("%s and %s have to sit in one column for the fill order to be forced",
              RACK_COLUMN_GOALS.get(i - 1), RACK_COLUMN_GOALS.get(i))
          .isEqualTo(map.xOf(goals[i - 1]));
      assertThat(map.yOf(goals[i - 1]) - map.yOf(goals[i]))
          .as("and one cell apart, which is the pitch every clearance sum below is measured"
              + " against")
          .isEqualTo(CELL_PITCH_MM);
    }
    for (int i = 0; i < goals.length; i++) {
      assertThat(map.predecessorsOf(goals[i]))
          .as("%s is reachable only from the cell above it and the cell below it: the sideways"
              + " links out of this column are all one-way outbound, so nothing enters a rack"
              + " cell from the side", RACK_COLUMN_GOALS.get(i))
          .hasSize(2);
    }
    assertThat(hopsTo(goals[2])[starts[2]])
        .as("the vehicle that has to go deepest also starts furthest away, which is what makes"
            + " this instance a fill-order problem rather than a queueing one")
        .isGreaterThan(hopsTo(goals[0])[starts[0]]);

    assertThat(footprint.maxReach() - 2 * footprint.sweptRadius(true))
        .as("the clearance the solver actually charges, recovered from the reach it publishes")
        .isEqualTo(SHIPPED_CLEARANCE_MM);
    assertThat(2 * footprint.sweptRadius(false))
        .as("so two bodies pivoting on adjacent cells share a budget of %s - %s = %s mm, and two"
            + " empty ones fit inside it", CELL_PITCH_MM, SHIPPED_CLEARANCE_MM,
            CELL_PITCH_MM - SHIPPED_CLEARANCE_MM)
        .isLessThan(CELL_PITCH_MM - SHIPPED_CLEARANCE_MM);
    assertThat(2 * footprint.sweptRadius(true))
        .as("and two loaded ones do not, so on this map a loaded vehicle can never turn beside"
            + " another loaded vehicle -- the convoy into the column cannot close up at any"
            + " cell where it has to change axis")
        .isGreaterThan(CELL_PITCH_MM - SHIPPED_CLEARANCE_MM);

    for (int agent = 0; agent < 3; agent++) {
      System.out.printf(Locale.ROOT,
          "[tie] a%d %s -> %s  work=%d  weighted=%d  hops=%d%n",
          agent, RACK_COLUMN_STARTS.get(agent), RACK_COLUMN_GOALS.get(agent),
          Roadmap.distanceAt(map.workDistanceTo(goals[agent]), starts[agent], Roadmap.DIR_NONE),
          map.distance(starts[agent], goals[agent]),
          hopsTo(goals[agent])[starts[agent]]);
      int[] work = map.workDistanceTo(goals[agent]);
      int[] turny = map.turnAwareDistanceTo(goals[agent]);
      for (int nose = 0; nose < Roadmap.NOSES; nose++) {
        System.out.printf(Locale.ROOT, "      nose=%d work=%d turnAware=%d%n",
            nose, Roadmap.distanceAt(work, starts[agent], nose),
            Roadmap.distanceAt(turny, starts[agent], nose));
      }
    }

    Lacam2Solver.Result result = solveAsProductionWould(
        intoTheColumn, SolverParams.WINDOW_FULL, noses, loaded);
    List<int[]> plan = result.solution();
    int[][] paths = transpose(plan, 3);
    System.out.print(
        planDump("one-shot, as production solves it: NONE, no loop budget, 1000ms", intoTheColumn, result));

    assertThat(result.reachedGoals())
        .as("planned unwindowed, so a plan that stops short of the three rack cells is a failure"
            + " rather than a truncation")
        .isTrue();
    assertThat(GridFixture.findCollisions(paths)).isEmpty();
    assertThat(GridFixture.findIllegalMoves(map, paths)).isEmpty();
    assertThat(footprintViolations(plan, noses, loaded))
        .as("and the bodies clear each other at every configuration, not merely the cells")
        .isEmpty();

    Lacam2Solver.Result windowed = solveAsProductionWould(
        intoTheColumn, SolverParams.DEFAULT_WINDOW, noses, loaded);
    System.out.print(planDump(
        "same instance through the joint fleet path, window=" + SolverParams.DEFAULT_WINDOW,
        intoTheColumn, windowed));

    Lacam2Solver.Result unladen = solveAsProductionWould(
        intoTheColumn, SolverParams.WINDOW_FULL, noses, new boolean[3]);
    System.out.print(planDump(
        "one-shot again with empty bodies, which makes the footprint inert",
        intoTheColumn, unladen));

    int shallowestIn = firstTickAt(paths[0], RACK_COLUMN_MOUTH);
    int middleIn = firstTickAt(paths[1], RACK_COLUMN_MOUTH);
    int deepestIn = firstTickAt(paths[2], RACK_COLUMN_MOUTH);
    int arrived = firstTickAt(paths[0], RACK_COLUMN_GOALS.get(0));
    int settled = settledOnGoalFrom(paths[0], goals[0]);

    assertThat(shallowestIn)
        .as("the column has one mouth, so the order through %s is the fill order, and the plan"
            + " gets it exactly backwards: the vehicle bound for the shallowest cell is nearest"
            + " and goes in first, ahead of the two it is standing in the way of. Nothing in the"
            + " cost model can see that -- an agent is ranked by its own distance to its own"
            + " goal, and %s is six hops from 3043 while %s is ten from 3045",
            RACK_COLUMN_MOUTH, RACK_COLUMN_STARTS.get(0), RACK_COLUMN_STARTS.get(2))
        .isLessThan(deepestIn)
        .isLessThan(middleIn);
    assertThat(arrived)
        .as("so 3043 is reached on the earliest tick its hop distance allows, which is the whole"
            + " problem rather than a success")
        .isEqualTo(hopsTo(goals[0])[starts[0]]);
    assertThat(settled)
        .as("and then the vehicle is pushed straight back off it: priority inheritance hands the"
            + " cell to whoever is queued behind, and a vehicle already parked on its goal has"
            + " no standing to refuse")
        .isGreaterThan(arrived);
    assertThat(offTheColumnAfter(paths[0], arrived))
        .as("the eviction is not a shuffle either. Every sideways link out of this column is"
            + " one-way outbound and the mouth is now full, so the only way back to 3043 is out"
            + " of the block, up the one-way return column and in through the mouth again")
        .isTrue();
    assertThat(settled - arrived)
        .as("which costs the round trip in full -- the vehicle stands on its own goal at t=%d,"
            + " leaves, and does not hold it again until t=%d", arrived, settled)
        .isGreaterThanOrEqualTo(2 * hopsTo(goals[0])[starts[0]]);

    assertThat(result.loopCount())
        .as("production runs the search with Objective.NONE and no loop budget, so it breaks at"
            + " the first solution the low level hands up: this plan is not what a budget"
            + " truncated, it is the only plan the search ever considered")
        .isLessThan(LOOP_LIMIT);
    assertThat(result.runtimeMs())
        .as("and it lands far inside the %dms production deadline, so the wall clock is not what"
            + " shaped it either", PRODUCTION_DEADLINE_MS)
        .isLessThan(PRODUCTION_DEADLINE_MS);
    assertThat(settledOnGoalFrom(transpose(unladen.solution(), 3)[0], goals[0]))
        .as("nor is it the body geometry: with empty bodies the footprint is inert at this pitch"
            + " and the same vehicle is still evicted from the same cell, so the 950mm budget"
            + " above changes how tightly the queue packs and nothing about who is in it")
        .isGreaterThan(firstTickAt(transpose(unladen.solution(), 3)[0], RACK_COLUMN_GOALS.get(0)));

    StringBuilder acrossSeeds = new StringBuilder(String.format(Locale.ROOT,
        "%n[rack column] one-shot as production solves it, across %d seeds%n"
            + "%-5s %9s %8s %8s %8s %11s %8s %6s%n",
        RACK_COLUMN_SEEDS,
        "seed", "makespan", "A@3043", "B@3044", "C@3045", "A first on", "evicted", "loops"));
    int evictions = 0;
    int shallowestFirstIn = 0;
    for (int seed = 0; seed < RACK_COLUMN_SEEDS; seed++) {
      Lacam2Solver.Result perSeed = solveAsProductionWould(
          new Instance(3, seed, starts, goals), SolverParams.WINDOW_FULL, noses, loaded);
      int[][] seedPaths = transpose(perSeed.solution(), 3);
      int reachedFirst = firstTickAt(seedPaths[0], RACK_COLUMN_GOALS.get(0));
      int heldFrom = settledOnGoalFrom(seedPaths[0], goals[0]);
      evictions += heldFrom > reachedFirst ? 1 : 0;
      shallowestFirstIn += firstTickAt(seedPaths[0], RACK_COLUMN_MOUTH) < firstTickAt(seedPaths[2], RACK_COLUMN_MOUTH)
          ? 1
          : 0;
      acrossSeeds.append(String.format(Locale.ROOT, "%-5d %9d %8d %8d %8d %11d %8b %6d%n",
          seed, perSeed.solution().size() - 1,
          heldFrom,
          settledOnGoalFrom(seedPaths[1], goals[1]),
          settledOnGoalFrom(seedPaths[2], goals[2]),
          reachedFirst, heldFrom > reachedFirst, perSeed.loopCount()));
    }
    System.out.print(acrossSeeds);

    assertThat(shallowestFirstIn)
        .as("the fill order is not a coin flip the tie-break RNG settles: across %d seeds the"
            + " vehicle bound for the shallowest cell reaches the mouth first every single time,"
            + " because it is nearest and the ranking asks nothing else",
            RACK_COLUMN_SEEDS)
        .isEqualTo(RACK_COLUMN_SEEDS);
    assertThat(evictions)
        .as("and it is pushed back off the cell it took every single time too, so this is the"
            + " shape of the instance rather than one unlucky rollout")
        .isEqualTo(RACK_COLUMN_SEEDS);

    StringBuilder guideAb = new StringBuilder(String.format(Locale.ROOT,
        "%n[rack column] LocalGuide A/B, one-shot, %d seeds%n%-5s %20s %20s %18s %18s%n",
        RACK_COLUMN_SEEDS,
        "seed", "makespan guide=on", "makespan guide=off", "A@3043 guide=on",
        "A@3043 guide=off"));
    int guideChangedTheOutcome = 0;
    for (int seed = 0; seed < RACK_COLUMN_SEEDS; seed++) {
      Instance perSeed = new Instance(3, seed, starts, goals);
      Lacam2Solver.Result guided = solveAsProductionWould(
          perSeed, SolverParams.WINDOW_FULL, noses, loaded, true);
      Lacam2Solver.Result blind = solveAsProductionWould(
          perSeed, SolverParams.WINDOW_FULL, noses, loaded, false);
      int guidedArrival = settledOnGoalFrom(transpose(guided.solution(), 3)[0], goals[0]);
      int blindArrival = settledOnGoalFrom(transpose(blind.solution(), 3)[0], goals[0]);
      guideChangedTheOutcome += guidedArrival == blindArrival ? 0 : 1;
      guideAb.append(String.format(Locale.ROOT, "%-5d %20d %20d %18d %18d%n",
          seed, guided.solution().size() - 1, blind.solution().size() - 1,
          guidedArrival, blindArrival));
    }
    System.out.print(guideAb);

    assertThat(map.sharesLaneWith(starts[0], goals[0]))
        .as("production seeds inGoalLane as isDroppingOff && sharesLaneWith(start, goal), and"
            + " that check only asks whether one axis runs monotonically from start to goal --"
            + " it does not stop at the lane mouth. %s sits on the pickup rack six cells above"
            + " the main aisle, entirely outside its drop-off lane, and still reads as inside it",
            RACK_COLUMN_STARTS.get(0))
        .isTrue();
    assertThat(map.sharesLaneWith(starts[1], goals[1]))
        .as("%s is on a different axis from its goal, so it gets no such boost",
            RACK_COLUMN_STARTS.get(1))
        .isFalse();
    assertThat(map.sharesLaneWith(starts[2], goals[2]))
        .as("%s neither", RACK_COLUMN_STARTS.get(2))
        .isFalse();

    AgentPriority[] asProductionSeedsThem = {
        new AgentPriority(0, 1, 0, 0, 0f), AgentPriority.NEWCOMER, AgentPriority.NEWCOMER };
    StringBuilder laneAb = new StringBuilder(String.format(Locale.ROOT,
        "%n[rack column] inGoalLane A/B, one-shot, %d seeds%n"
            + "%-5s %9s %9s %12s %12s %11s %11s %9s %9s%n",
        RACK_COLUMN_SEEDS, "seed", "span off", "span on", "A first off", "A first on",
        "A held off", "A held on", "evict off", "evict on"));
    int worseWithTheBoost = 0;
    for (int seed = 0; seed < RACK_COLUMN_SEEDS; seed++) {
      Instance perSeed = new Instance(3, seed, starts, goals);
      Lacam2Solver.Result off = solveAsProductionWould(
          perSeed, SolverParams.WINDOW_FULL, noses, loaded, true, null);
      Lacam2Solver.Result on = solveAsProductionWould(
          perSeed, SolverParams.WINDOW_FULL, noses, loaded, true, asProductionSeedsThem);
      int offArrival = settledOnGoalFrom(transpose(off.solution(), 3)[0], goals[0]);
      int onArrival = settledOnGoalFrom(transpose(on.solution(), 3)[0], goals[0]);
      worseWithTheBoost += onArrival > offArrival ? 1 : 0;
      int offFirst = firstTickAt(transpose(off.solution(), 3)[0], RACK_COLUMN_GOALS.get(0));
      int onFirst = firstTickAt(transpose(on.solution(), 3)[0], RACK_COLUMN_GOALS.get(0));
      laneAb.append(String.format(Locale.ROOT,
          "%-5d %9d %9d %12d %12d %11d %11d %9b %9b%n",
          seed, off.solution().size() - 1, on.solution().size() - 1, offFirst, onFirst,
          offArrival, onArrival, offArrival > offFirst, onArrival > onFirst));
    }
    System.out.print(laneAb);

    AgentPriority[] rankedByHops = new AgentPriority[3];
    for (int agent = 0; agent < 3; agent++) {
      rankedByHops[agent] = new AgentPriority(
          0, 0, hopsTo(goals[agent])[starts[agent]], 0, 0f);
    }
    StringBuilder hopAb = new StringBuilder(String.format(Locale.ROOT,
        "%n[rack column] priority by weighted cost vs by hops, one-shot, %d seeds%n"
            + "  hops: A=%d B=%d C=%d   weighted: A=%d B=%d C=%d%n"
            + "%-5s %10s %10s %13s %13s %10s %10s%n",
        RACK_COLUMN_SEEDS,
        hopsTo(goals[0])[starts[0]], hopsTo(goals[1])[starts[1]], hopsTo(goals[2])[starts[2]],
        map.distance(starts[0], goals[0]), map.distance(starts[1], goals[1]),
        map.distance(starts[2], goals[2]),
        "seed", "span cost", "span hops", "A first cost", "A first hops", "A held cost",
        "A held hops"));
    int orderedDeepestFirst = 0;
    for (int seed = 0; seed < RACK_COLUMN_SEEDS; seed++) {
      Instance perSeed = new Instance(3, seed, starts, goals);
      Lacam2Solver.Result byCost = solveAsProductionWould(
          perSeed, SolverParams.WINDOW_FULL, noses, loaded, true, null);
      Lacam2Solver.Result byHops = solveAsProductionWould(
          perSeed, SolverParams.WINDOW_FULL, noses, loaded, true, rankedByHops);
      int[][] costPaths = transpose(byCost.solution(), 3);
      int[][] hopPaths = transpose(byHops.solution(), 3);
      if (firstTickAt(hopPaths[2], RACK_COLUMN_MOUTH) < firstTickAt(hopPaths[0],
          RACK_COLUMN_MOUTH)) {
        orderedDeepestFirst++;
      }
      hopAb.append(String.format(Locale.ROOT, "%-5d %10d %10d %13d %13d %10d %10d%n",
          seed, byCost.solution().size() - 1, byHops.solution().size() - 1,
          firstTickAt(costPaths[0], RACK_COLUMN_GOALS.get(0)),
          firstTickAt(hopPaths[0], RACK_COLUMN_GOALS.get(0)),
          settledOnGoalFrom(costPaths[0], goals[0]),
          settledOnGoalFrom(hopPaths[0], goals[0])));
    }
    hopAb.append(String.format(Locale.ROOT,
        "  seeds where the deepest vehicle reached the mouth first: %d/%d%n",
        orderedDeepestFirst, RACK_COLUMN_SEEDS));
    System.out.print(hopAb);

    System.out.print(guidePathDump("root guide paths, guide on", result));
    System.out.print(guidePathDump("root guide paths, guide off",
        solveAsProductionWould(intoTheColumn, SolverParams.WINDOW_FULL, noses, loaded, false)));

    for (int seed : new int[] { 0, 4 }) {
      Instance perSeed = new Instance(3, seed, starts, goals);
      Lacam2Solver.Result byHops = solveAsProductionWould(
          perSeed, SolverParams.WINDOW_FULL, noses, loaded, true, rankedByHops);
      System.out.print(guidePathDump(
          "root guide paths, priority by hops, seed " + seed, byHops));
      System.out.print(planDump(
          "priority by hops, seed " + seed, perSeed, byHops));
      System.out.printf(Locale.ROOT,
          "  weightOf(0110)=%d weightOf(0084)=%d weightOf(0009)=%d weightOf(0082)=%d"
              + " weightOf(0013)=%d%n",
          map.weightOf(map.idOf("0110")), map.weightOf(map.idOf("0084")),
          map.weightOf(map.idOf("0009")), map.weightOf(map.idOf("0082")),
          map.weightOf(map.idOf("0013")));
    }

    for (int refine : new int[] { 1, 2, 3 }) {
      for (boolean byHops : new boolean[] { false, true }) {
        Instance perSeed = new Instance(3, 0, starts, goals);
        Lacam2Solver.Result probe = solveAsProductionWould(
            perSeed, SolverParams.WINDOW_FULL, noses, loaded, true,
            byHops ? rankedByHops : null, refine);
        int[][] probePaths = transpose(probe.solution(), 3);
        System.out.print(guidePathDump(String.format(Locale.ROOT,
            "numRefine=%d priority=%s | makespan=%d A first on 3043=%d A held from=%d",
            refine, byHops ? "hops" : "weighted cost", probe.solution().size() - 1,
            firstTickAt(probePaths[0], RACK_COLUMN_GOALS.get(0)),
            settledOnGoalFrom(probePaths[0], goals[0])), probe));
      }
    }

    StringBuilder cgf = new StringBuilder(String.format(Locale.ROOT,
        "%n[rack column] clearGoalFirst A/B, one-shot, %d seeds%n%-5s %12s %12s %14s %14s%n",
        RACK_COLUMN_SEEDS, "seed", "span on", "span off", "A first on", "A first off"));
    for (int seed = 0; seed < RACK_COLUMN_SEEDS; seed++) {
      Instance perSeed = new Instance(3, seed, starts, goals);
      String before = System.getProperty(CLEAR_GOAL_PROPERTY);
      System.setProperty(CLEAR_GOAL_PROPERTY, "true");
      Lacam2Solver.Result cgfOn = solveAsProductionWould(
          perSeed, SolverParams.WINDOW_FULL, noses, loaded, true, null);
      System.setProperty(CLEAR_GOAL_PROPERTY, "false");
      Lacam2Solver.Result cgfOff = solveAsProductionWould(
          perSeed, SolverParams.WINDOW_FULL, noses, loaded, true, null);
      restore(CLEAR_GOAL_PROPERTY, before);
      cgf.append(String.format(Locale.ROOT, "%-5d %12d %12d %14d %14d%n",
          seed, cgfOn.solution().size() - 1, cgfOff.solution().size() - 1,
          firstTickAt(transpose(cgfOn.solution(), 3)[0], RACK_COLUMN_GOALS.get(0)),
          firstTickAt(transpose(cgfOff.solution(), 3)[0], RACK_COLUMN_GOALS.get(0))));
    }
    System.out.print(cgf);

    Roadmap twoWayAisles = withTwoWaySideLinks();
    for (String cell : List.of("3042", "3043", "3044", "3045", "3046", "3047")) {
      assertThat(twoWayAisles.neighborsOf(twoWayAisles.idOf(cell)))
          .as("%s must now be able to step sideways and come back", cell)
          .isNotEmpty();
    }
    assertThat(sideLinkIsTwoWay(twoWayAisles, "3043", "3051"))
        .as("the whole point of this variant is that the sideways link is no longer one-way")
        .isTrue();
    assertThat(sideLinkIsTwoWay(map, "3043", "3051"))
        .as("while on the shipped map it is")
        .isFalse();

    Lacam2Solver.Result sidestep = new Lacam2Solver(twoWayAisles).solve(
        new MapfInstance(idsIn(twoWayAisles, RACK_COLUMN_STARTS),
            idsIn(twoWayAisles, RACK_COLUMN_GOALS)),
        PRODUCTION_DEADLINE_MS, 0, Lacam2Solver.Objective.NONE,
        Lacam2Solver.DEFAULT_RESTART_RATE, Lacam2Solver.LOOP_LIMIT_UNBOUNDED,
        SolverParams.WINDOW_FULL, null, noses, loaded, null, null,
        StaticBodies.NONE);
    System.out.printf(Locale.ROOT,
        "%n[two-way side links] ticks=%d reachedGoals=%b loops=%d ms=%d%n",
        sidestep.solution().size() - 1, sidestep.reachedGoals(), sidestep.loopCount(),
        sidestep.runtimeMs());
    for (int agent = 0; agent < 3; agent++) {
      System.out.printf(Locale.ROOT, "  %s -> %-6s %s%n",
          RACK_COLUMN_STARTS.get(agent), RACK_COLUMN_GOALS.get(agent),
          routeOf(twoWayAisles, sidestep, agent));
    }

    for (int agent = 0; agent < 3; agent++) {
      assertThat(routeOf(result, agent))
          .as("agent %d does reach its own cell in the end, so this is a cost failure and not a"
              + " feasibility one", agent)
          .endsWith(RACK_COLUMN_GOALS.get(agent));
    }
    assertThat(plan.size() - 1)
        .as("the deepest vehicle needs %d ticks and has no slack, so that is the makespan an"
            + " ordered plan would hit; this one pays the eviction on top",
            hopsTo(goals[2])[starts[2]])
        .isGreaterThan(hopsTo(goals[2])[starts[2]]);
  }

  private static int settledOnGoalFrom(int[] path, int goal) {
    int tick = path.length;
    while (tick > 0 && path[tick - 1] == goal) {
      tick--;
    }
    return tick;
  }

  private static boolean offTheColumnAfter(int[] path, int fromTick) {
    double column = map.xOf(map.idOf(RACK_COLUMN_MOUTH));
    for (int tick = fromTick; tick < path.length; tick++) {
      if (map.xOf(path[tick]) != column) {
        return true;
      }
    }
    return false;
  }

  private static String guidePathDump(String title, Lacam2Solver.Result result) {
    StringBuilder out = new StringBuilder(String.format(Locale.ROOT, "%n[guide] %s%n", title));
    int[][] paths = result.guidePaths();
    if (paths == null) {
      return out.append("  (none)").append(System.lineSeparator()).toString();
    }
    for (int agent = 0; agent < paths.length; agent++) {
      out.append(String.format(Locale.ROOT, "  a%d %s%n", agent, runLengthOf(paths[agent])));
    }
    return out.toString();
  }

  private static String runLengthOf(int[] path) {
    if (path == null) {
      return "(null)";
    }
    StringBuilder out = new StringBuilder();
    int at = 0;
    while (at < path.length) {
      int run = 1;
      while (at + run < path.length && path[at + run] == path[at]) {
        run++;
      }
      if (out.length() > 0) {
        out.append(" -> ");
      }
      out.append(path[at] < 0 ? "?" : map.nameOf(path[at]));
      if (run > 1) {
        out.append("x").append(run);
      }
      at += run;
    }
    return out.toString();
  }

  private static Roadmap withTwoWaySideLinks() throws Exception {
    Roadmap.Model shipped = RoadmapXmlLoader.load(RoadmapXmlLoader.findMap(MAP_FILE));
    List<Roadmap.Path> paths = new ArrayList<>(shipped.paths());
    for (Roadmap.Path path : shipped.paths()) {
      if (path.reverse()) {
        continue;
      }
      double fromX = map.xOf(map.idOf(path.src()));
      double toX = map.xOf(map.idOf(path.dst()));
      if (fromX == 37000 && toX == 38000) {
        paths.add(new Roadmap.Path(
            path.name() + "~opened", path.dst(), path.src(), path.length(), false, true));
      }
    }
    return new Roadmap(new Roadmap.Model(
        shipped.points(), paths, shipped.cargoSlotPoints(), shipped.dropOffPoints()));
  }

  private static boolean sideLinkIsTwoWay(Roadmap roadmap, String from, String to) {
    boolean forward = false;
    boolean back = false;
    for (int neighbour : roadmap.neighborsOf(roadmap.idOf(from))) {
      forward |= neighbour == roadmap.idOf(to);
    }
    for (int neighbour : roadmap.neighborsOf(roadmap.idOf(to))) {
      back |= neighbour == roadmap.idOf(from);
    }
    return forward && back;
  }

  private static int[] idsIn(Roadmap roadmap, List<String> points) {
    int[] ids = new int[points.size()];
    for (int i = 0; i < ids.length; i++) {
      ids[i] = roadmap.idOf(points.get(i));
    }
    return ids;
  }

  private static int firstTickAt(int[] path, String point) {
    int vertex = map.idOf(point);
    for (int tick = 0; tick < path.length; tick++) {
      if (path[tick] == vertex) {
        return tick;
      }
    }
    return Integer.MAX_VALUE;
  }

  private static Lacam2Solver.Result solveAsProductionWould(
      Instance instance, int window, int[] startAxis, boolean[] loaded) {
    return solveAsProductionWould(instance, window, startAxis, loaded, true);
  }

  private static Lacam2Solver.Result solveAsProductionWould(
      Instance instance, int window, int[] startAxis, boolean[] loaded, boolean guided) {
    return solveAsProductionWould(instance, window, startAxis, loaded, guided, null);
  }

  private static Lacam2Solver.Result solveAsProductionWould(
      Instance instance, int window, int[] startAxis, boolean[] loaded, boolean guided,
      AgentPriority[] rootPriorities) {
    return solveAsProductionWould(
        instance, window, startAxis, loaded, guided, rootPriorities, null);
  }

  private static Lacam2Solver.Result solveAsProductionWould(
      Instance instance, int window, int[] startAxis, boolean[] loaded, boolean guided,
      AgentPriority[] rootPriorities, Integer numRefine) {
    String footprintBefore = System.getProperty(FOOTPRINT_PROPERTY);
    String guideBefore = System.getProperty(GUIDE_PROPERTY);
    String refineBefore = System.getProperty(REFINE_PROPERTY);
    System.setProperty(FOOTPRINT_PROPERTY, "true");
    System.setProperty(GUIDE_PROPERTY, String.valueOf(guided));
    if (numRefine != null) {
      System.setProperty(REFINE_PROPERTY, String.valueOf(numRefine));
    }
    try {
      return new Lacam2Solver(map).solve(
          new MapfInstance(instance.starts(), instance.goals()),
          PRODUCTION_DEADLINE_MS,
          instance.seed(),
          Lacam2Solver.Objective.NONE,
          Lacam2Solver.DEFAULT_RESTART_RATE,
          Lacam2Solver.LOOP_LIMIT_UNBOUNDED,
          window,
          rootPriorities,
          startAxis,
          loaded,
          null,
          null,
          StaticBodies.NONE);
    } finally {
      restore(FOOTPRINT_PROPERTY, footprintBefore);
      restore(GUIDE_PROPERTY, guideBefore);
      restore(REFINE_PROPERTY, refineBefore);
    }
  }

  private static void restore(String key, String previous) {
    if (previous == null) {
      System.clearProperty(key);
    } else {
      System.setProperty(key, previous);
    }
  }

  private static List<String> footprintViolations(
      List<int[]> plan, int[] startAxis, boolean[] loaded) {
    Footprint footprint = new Footprint(map);
    int[] nose = startAxis.clone();
    List<String> clashes = new ArrayList<>();
    for (int tick = 0; tick + 1 < plan.size(); tick++) {
      Footprint.Motion[] motions = new Footprint.Motion[nose.length];
      for (int agent = 0; agent < nose.length; agent++) {
        motions[agent] = new Footprint.Motion(
            plan.get(tick)[agent], plan.get(tick + 1)[agent], nose[agent], loaded[agent]);
      }
      for (int ego = 0; ego < motions.length; ego++) {
        for (int other = ego + 1; other < motions.length; other++) {
          if (footprint.collideMoving(motions[ego], motions[other])) {
            clashes.add(String.format(Locale.ROOT, "t=%d: %d(%s->%s) clashes with %d(%s->%s)",
                tick, ego, map.nameOf(motions[ego].from()), map.nameOf(motions[ego].to()),
                other, map.nameOf(motions[other].from()), map.nameOf(motions[other].to())));
          }
        }
      }
      for (int agent = 0; agent < nose.length; agent++) {
        nose[agent] = footprint.noseAfter(
            nose[agent], plan.get(tick)[agent], plan.get(tick + 1)[agent]);
      }
    }
    return clashes;
  }

  @Test
  void planning3130To3106FiveTimesShowsWhatEachSeedPicks() {
    int start = map.idOf("3130");
    int goal = map.idOf("3106");
    int[] hops = hopsTo(goal);

    List<List<String>> bySeed = new ArrayList<>();
    for (int seed = 0; seed < 5; seed++) {
      Lacam2Solver.Result result = solve(
          new Instance(1, seed, new int[] { start }, new int[] { goal }), SolverParams.WINDOW_FULL);
      List<String> route = routeOf(result, 0);
      bySeed.add(route);
      System.out.printf(Locale.ROOT,
          "[3130->3106] seed=%d steps=%2d hopBound=%2d weighted=%2d surcharge=%2d turns=%d"
              + " arrived=%-5b drainedOpen=%-5b loops=%4d nodes=%5d ms=%d%n",
          seed, route.size() - 1, hops[start], map.distance(start, goal), surchargePaidOn(route),
          turnsIn(route), result.reachedGoals(), result.optimal(), result.loopCount(),
          result.nodesGenerated(), result.runtimeMs());
      System.out.printf(Locale.ROOT, "[3130->3106] seed=%d route=%s%n",
          seed, String.join(" -> ", route));
      System.out.printf(Locale.ROOT, "[3130->3106] seed=%d cargoPointsOnRoute=%s%n",
          seed, surchargedPointsIn(route));
    }

    for (int seed = 0; seed < bySeed.size(); seed++) {
      assertThat(bySeed.get(seed))
          .as("seed %d: one vehicle with nobody to yield to has to walk the whole way, so a"
              + " route that stops short is a planning failure rather than a truncation", seed)
          .startsWith("3130")
          .endsWith("3106");
    }
  }

  private static List<String> surchargedPointsIn(List<String> route) {
    List<String> surcharged = new ArrayList<>();
    for (String point : route) {
      if (map.weightOf(map.idOf(point)) > 1) {
        surcharged.add(point + "(w=" + map.weightOf(map.idOf(point)) + ")");
      }
    }
    return surcharged;
  }

  private static int surchargePaidOn(List<String> route) {
    return surchargePaidOn(map, route);
  }

  private static int surchargePaidOn(Roadmap roadmap, List<String> route) {
    int paid = 0;
    for (int i = 1; i < route.size(); i++) {
      paid += roadmap.weightOf(roadmap.idOf(route.get(i))) - 1;
    }
    return paid;
  }

  private static final String TURN_BLOCK_START = "0326";
  private static final String TURN_BLOCK_GOAL = "0006";
  private static final List<String> ACROSS_THEN_DOWN = List.of("0326", "0325", "0324", "0006");
  private static final List<String> DOWN_THEN_ACROSS = List.of("0326", "0045", "0005", "0006");
  private static final List<String> DOGLEG = List.of("0326", "0325", "0005", "0006");
  private static final int TURN_BLOCK_SEEDS = 24;
  private static final String FOOTPRINT_PROPERTY = "fms.pibt.footprint";

  private static List<String> routesAcrossSeeds(
      Roadmap roadmap, int start, int goal, int startAxis) {
    List<String> chosen = new ArrayList<>();
    for (int seed = 0; seed < TURN_BLOCK_SEEDS; seed++) {
      chosen.add(routeWithStartAxis(roadmap, start, goal, startAxis, seed).toString());
    }
    return chosen;
  }

  private static List<String> routeWithStartAxis(
      Roadmap roadmap, int start, int goal, int startAxis, int seed) {
    Lacam2Solver.Result result = new Lacam2Solver(roadmap).solve(
        new MapfInstance(new int[] { start }, new int[] { goal }),
        DEADLINE_MS,
        seed,
        Lacam2Solver.Objective.SUM_OF_LOSS,
        Lacam2Solver.DEFAULT_RESTART_RATE,
        LOOP_LIMIT,
        SolverParams.WINDOW_FULL,
        null,
        new int[] { startAxis },
        new boolean[] { false },
        null,
        null,
        StaticBodies.NONE);
    return routeOf(roadmap, result, 0);
  }

  private static boolean everyHopIsARealEdge(Roadmap roadmap, List<String> route) {
    for (int i = 0; i + 1 < route.size(); i++) {
      int from = roadmap.idOf(route.get(i));
      int to = roadmap.idOf(route.get(i + 1));
      boolean linked = false;
      for (int neighbour : roadmap.neighborsOf(from)) {
        linked |= neighbour == to;
      }
      if (!linked) {
        return false;
      }
    }
    return true;
  }

  private static Instance loneTrip(Roadmap roadmap, String start, String goal) {
    return new Instance(1, 0, new int[] { roadmap.idOf(start) }, new int[] { roadmap.idOf(goal) });
  }

  @Test
  void anOutboundVehicleClimbsToThePickupLaneWithinAFewHopsAndNeverComesBackDown() {
    int park = map.idOf("0047");
    int pickupLaneFeeder = map.idOf("0082");
    int dropoffLaneFeeder = map.idOf("0004");

    for (int seed = 0; seed < 4; seed++) {
      List<String> outbound = routeOf(
          solve(new Instance(1, seed, new int[] { park }, new int[] { pickupLaneFeeder }),
              SolverParams.WINDOW_FULL),
          0);
      List<String> home = routeOf(
          solve(new Instance(1, seed, new int[] { dropoffLaneFeeder }, new int[] { park }),
              SolverParams.WINDOW_FULL),
          0);

      assertThat(pointsAt(outbound, LOWER_MAINLINE))
          .as("seed %d: riding east on the lower lane and climbing later ties with climbing now,"
              + " so the crossing point is a coin flip -- but the upper lane absorbs, because"
              + " once up there stepping back down is a strict unit worse", seed)
          .isLessThanOrEqualTo(5);
      assertThat(pointsAt(outbound, UPPER_MAINLINE))
          .as("seed %d: so almost the whole outbound run happens on the pickup lane", seed)
          .isGreaterThan(outbound.size() - 6);
      assertThat(pointsAt(home, UPPER_MAINLINE))
          .as("seed %d: the return leg has no coin to flip at all -- it starts and ends on the"
              + " lower lane, and leaving it could only cost more", seed)
          .isZero();
    }
  }

  @Test
  void everyWindowedSolveSpendsItsWholeLoopBudgetSoQualityIsBudgetBoundNotSeedBound() {
    for (Quality quality : windowedPlans) {
      assertThat(solve(quality.instance(), WINDOW).loopCount())
          .as("%s: OPEN never drains inside the window, so the plan handed back is whatever the"
              + " budget bought; raising the budget is the lever on windowed quality, and the"
              + " tie-break seed is not one", label(quality))
          .isEqualTo(LOOP_LIMIT);
    }
  }

  @Test
  void aOneShotPlanForALoneVehicleNeverLeavesAPointOnlyToComeBackToIt() {
    int[] core = stronglyConnectedCore(map);
    int planned = 0;
    List<String> offenders = new ArrayList<>();
    for (int trial = 0; trial < 60; trial++) {
      Instance lone = instanceOn(core, 1, 1000 + trial);
      Lacam2Solver.Result result = solve(lone, SolverParams.WINDOW_FULL);
      if (!result.reachedGoals()) {
        continue;
      }
      planned++;
      int[] kinds = backstepKinds(transpose(result.solution(), 1));
      if (kinds[0] + kinds[1] > 0) {
        offenders.add(routeOf(result, 0).toString());
      }
    }

    assertThat(planned).as("the corpus has to actually plan something").isGreaterThan(50);
    assertThat(offenders)
        .as("with nobody to yield to, every excursion away from a point and back is waste the"
            + " solver invented on its own; oscillation seen on the floor therefore comes from"
            + " traffic or from replanning, not from one-shot search")
        .isEmpty();
  }

  @Test
  void oneShotPlansAlreadyOscillateOnceTheFleetIsDense() {
    int wobbles = 0;
    for (Quality quality : sprintPlans) {
      wobbles += quality.wobblingBacksteps();
    }

    assertThat(wobbles)
        .as("one-shot search has the whole horizon in view and still leaves vehicles stepping"
            + " off a point and back while nobody else uses it, so oscillation seen on the"
            + " floor cannot be blamed on windowed replanning alone")
        .isPositive();
  }

  @Test
  void pointlessExcursionsAreAlmostAlwaysAStepTowardsTheGoalThatHadToBeGivenBack() {
    int towardsTheGoal = 0;
    int awayFromIt = 0;
    int longerThanTheWindow = 0;
    for (Quality quality : sprintPlans) {
      int[][] paths = transpose(quality.plan(), quality.instance().agents());
      for (int agent = 0; agent < paths.length; agent++) {
        for (int[] excursion : pointlessExcursionsOf(paths, agent)) {
          int goal = quality.instance().goals()[agent];
          int before = map.distance(paths[agent][excursion[0]], goal);
          int after = map.distance(paths[agent][excursion[1]], goal);
          towardsTheGoal += after < before ? 1 : 0;
          awayFromIt += after > before ? 1 : 0;
          longerThanTheWindow += excursion[2] - excursion[1] > WINDOW ? 1 : 0;
        }
      }
    }

    assertThat(towardsTheGoal)
        .as("a vehicle stepping backwards for nobody would be the solver inventing waste; what"
            + " actually happens is it takes a step that is locally correct and then has to"
            + " hand it back, which is a lookahead failure rather than a routing one")
        .isGreaterThan(awayFromIt * 4);
    assertThat(longerThanTheWindow)
        .as("these excursions resolve well inside the planning window, so the conflict that"
            + " causes them was visible to the search when it chose to advance")
        .isLessThanOrEqualTo(1);
  }

  private static List<int[]> pointlessExcursionsOf(int[][] paths, int agent) {
    List<int[]> found = new ArrayList<>();
    int[] path = paths[agent];
    int at = 0;
    while (true) {
      int left = nextChange(path, at);
      int returned = left < 0 ? -1 : nextChange(path, left);
      if (returned < 0) {
        return found;
      }
      if (path[returned] != path[at]) {
        at = left;
        continue;
      }
      if (!visitedByOther(paths, agent, path[at], left, returned)) {
        found.add(new int[] { at, left, returned });
      }
      at = returned;
    }
  }

  private static final String[] REAL_GOALS = {
      "0047", "0048", "0240", "0250", "0260", "0270",
      "1001", "1002", "1003", "1004", "1005", "1006",
      "1010", "1011", "1012",
      "0200", "0210", "0220", "0230", "1000", "1007", "1008", "1009" };

  record Revisit(int loopLength, int vertex, int leftAt, int returnedAt, List<Integer> loop) {
  }

  record Run(List<int[]> trajectory, boolean solverGaveNothing, int cycles, long solveMs) {
  }

  @Test
  void theSearchBudgetRatherThanTheWallClockDecidesWhenPlanningStops() {
    for (Quality quality : allPlans()) {
      assertThat(quality.runtimeMs())
          .as("%s: the deadline is a liveness backstop; once it is what ends the search, every"
              + " number in the report becomes a property of this machine's load", label(quality))
          .isLessThan(DEADLINE_MS / 4);
    }
  }

  @Test
  void replanningAnInstanceReproducesItConfigurationByConfiguration() {
    for (Quality quality : allPlans()) {
      List<int[]> replanned = solve(quality.instance(), quality.window()).solution();

      assertThat(replanned).as("%s", label(quality)).hasSameSizeAs(quality.plan());
      for (int t = 0; t < replanned.size(); t++) {
        assertThat(replanned.get(t))
            .as("%s @t=%d", label(quality), t)
            .containsExactly(quality.plan().get(t));
      }
    }
  }

  @Test
  void windowedPlansSpendTheWindowClosingDistanceToTheGoal() {
    for (Quality quality : windowedPlans) {
      assertThat(quality.progressRate())
          .as("%s: fraction of the window's reachable distance that the plan actually closed",
              label(quality))
          .isGreaterThanOrEqualTo(MIN_PROGRESS_RATE);
    }
  }

  @Test
  void vehiclesRarelyLeaveAPointOnlyToComeStraightBackForNobody() {
    for (Quality quality : allPlans()) {
      assertThat(quality.wobbleRate())
          .as("%s: wobbles per agent-tick. Leaving a point and returning to it is only work when"
              + " another vehicle used the vacated point meanwhile; counting wobbles as a share"
              + " of all backsteps would swing between 0 and 1 on single-digit tallies",
              label(quality))
          .isLessThanOrEqualTo(MAX_WOBBLE_RATE);
    }
  }

  @Test
  void noFleetLeavesALargeShareOfItsVehiclesWithoutNetProgress() {
    for (Quality quality : windowedPlans) {
      assertThat(quality.stalledShare())
          .as("%s: vehicles that ended the window no nearer their goal than they started",
              label(quality))
          .isLessThanOrEqualTo(MAX_STALLED_SHARE);
    }
  }

  @Test
  void vehiclesShortOfTheirGoalDoNotSpendMostOfTheirTicksStandingStill() {
    for (Quality quality : windowedPlans) {
      assertThat(quality.waitRate())
          .as("%s: share of off-goal ticks spent waiting", label(quality))
          .isLessThanOrEqualTo(MAX_WAIT_RATE);
    }
  }

  @Test
  void sprintPlansStayWithinAKnownFactorOfTheIndependentShortestPathBound() {
    for (Quality quality : sprintPlans) {
      assertThat(quality.suboptimality())
          .as("%s: sum-of-costs against the sum of independent shortest paths, which is a"
              + " lower bound because it ignores every interaction between vehicles",
              label(quality))
          .isLessThanOrEqualTo(MAX_SUBOPTIMALITY);
    }
  }

  private static List<Quality> allPlans() {
    List<Quality> all = new ArrayList<>(windowedPlans);
    all.addAll(sprintPlans);
    return all;
  }

  @Test
  void twoAgentsSentToTheSameSharedEgressHaveNoSolutionAndTheSolverMustSaySo() {
    String westLane = "3098";
    String sharedEgress = "3108";
    String eastLane = "3118";

    for (String point : List.of(westLane, sharedEgress, eastLane)) {
      assumeTrue(map.has(point), point + " is missing from " + MAP_FILE);
    }
    assertThat(map.neighborsOf(map.idOf(westLane)))
        .as("%s is only a shared egress if %s can reach it", sharedEgress, westLane)
        .contains(map.idOf(sharedEgress));
    assertThat(map.neighborsOf(map.idOf(eastLane)))
        .as("%s is only a shared egress if %s can reach it", sharedEgress, eastLane)
        .contains(map.idOf(sharedEgress));

    Instance bothAimedAtOneCell = new Instance(
        2,
        1,
        new int[] { map.idOf(westLane), map.idOf(eastLane) },
        new int[] { map.idOf(sharedEgress), map.idOf(sharedEgress) });

    Lacam2Solver.Result result = solve(bothAimedAtOneCell, WINDOW);

    assertThat(result.solved())
        .as("both vehicles were aimed at %s and one cell cannot hold two of them, so this instance"
            + " has no solution at all; the solver must report that rather than emit a plan in"
            + " which the loser is pushed off and keeps returning%n"
            + "      from %s: %s%n"
            + "      from %s: %s",
            sharedEgress,
            westLane, compressedRouteOf(result, 0),
            eastLane, compressedRouteOf(result, 1))
        .isFalse();
  }

  private static List<String> compressedRouteOf(Lacam2Solver.Result result, int agent) {
    List<String> route = new ArrayList<>();
    for (int[] config : result.solution()) {
      String point = map.nameOf(config[agent]);
      if (route.isEmpty() || !route.get(route.size() - 1).equals(point)) {
        route.add(point);
      }
    }
    return route;
  }

  private static Quality planAndScore(Instance instance, int window) {
    return score(instance, window, solve(instance, window));
  }

  private static Lacam2Solver.Result solve(Instance instance, int window) {
    return solve(map, instance, window);
  }

  private static Lacam2Solver.Result solve(Roadmap roadmap, Instance instance, int window) {
    return solve(roadmap, instance, window, LOOP_LIMIT, DEADLINE_MS);
  }

  private static Lacam2Solver.Result solve(
      Roadmap roadmap, Instance instance, int window, int loopLimit, long deadlineMs) {
    return new Lacam2Solver(roadmap).solve(
        new MapfInstance(instance.starts(), instance.goals()),
        deadlineMs,
        instance.seed(),
        Lacam2Solver.Objective.SUM_OF_LOSS,
        Lacam2Solver.DEFAULT_RESTART_RATE,
        loopLimit,
        window,
        null,
        null,
        null,
        null,
        null,
        StaticBodies.NONE);
  }

  private static Quality score(Instance instance, int window, Lacam2Solver.Result result) {
    List<int[]> plan = result.solution();
    int[][] paths = transpose(plan, instance.agents());
    int[] starts = instance.starts();
    int[] goals = instance.goals();
    boolean unwindowed = window == SolverParams.WINDOW_FULL;

    long lowerBound = 0;
    long progressMade = 0;
    long progressAvailable = 0;
    int stalled = 0;
    for (int agent = 0; agent < instance.agents(); agent++) {
      int[] hops = hopsTo(goals[agent]);
      int atStart = hops[starts[agent]];
      int atEnd = plan.isEmpty() ? atStart : hops[paths[agent][paths[agent].length - 1]];
      lowerBound += atStart;
      progressMade += atStart - atEnd;
      progressAvailable += unwindowed ? atStart : Math.min(atStart, window);
      if (atStart > 0 && atEnd >= atStart) {
        stalled++;
      }
    }

    int[] backsteps = backstepKinds(paths);
    long[] ticks = waitAndOffGoalTicks(paths, goals);
    return new Quality(
        instance,
        window,
        Math.max(0, plan.size() - 1),
        plan,
        violationsIn(instance, plan, paths, unwindowed),
        result.reachedGoals(),
        lowerBound,
        sumOfCosts(paths, goals),
        progressMade,
        progressAvailable,
        backsteps[0],
        backsteps[1],
        ticks[0],
        ticks[1],
        stalled,
        result.runtimeMs());
  }

  private static List<String> violationsIn(
      Instance instance, List<int[]> plan, int[][] paths, boolean mustReachGoals) {
    List<String> issues = new ArrayList<>();
    if (plan.isEmpty()) {
      issues.add("empty plan");
      return issues;
    }
    if (!Arrays.equals(plan.get(0), instance.starts())) {
      issues.add("plan does not begin at the starts");
    }
    if (mustReachGoals && !Arrays.equals(plan.get(plan.size() - 1), instance.goals())) {
      issues.add("plan does not end at the goals");
    }
    issues.addAll(GridFixture.findCollisions(paths));
    issues.addAll(GridFixture.findIllegalMoves(map, paths));
    return issues;
  }

  private static int pointsAt(List<String> route, double lane) {
    int count = 0;
    for (String point : route) {
      count += map.yOf(map.idOf(point)) == lane ? 1 : 0;
    }
    return count;
  }

  private static int costThrough(int candidate, int goal) {
    return costThrough(map, candidate, goal);
  }

  private static int costThrough(Roadmap roadmap, int candidate, int goal) {
    return roadmap.weightOf(candidate) + roadmap.distance(candidate, goal);
  }

  private static List<Integer> pointsOn(double lane) {
    List<Integer> points = new ArrayList<>();
    for (int point = 0; point < map.size(); point++) {
      if (map.yOf(point) == lane) {
        points.add(point);
      }
    }
    points.sort((a, b) -> Double.compare(map.xOf(a), map.xOf(b)));
    return points;
  }

  private static int ticksOn(int[][] paths, double lane) {
    int ticks = 0;
    for (int[] path : paths) {
      for (int vertex : path) {
        ticks += map.yOf(vertex) == lane ? 1 : 0;
      }
    }
    return ticks;
  }

  private static int cargoSlotsOn(double lane) {
    int surcharged = 0;
    for (int point = 0; point < map.size(); point++) {
      if (map.yOf(point) == lane && map.weightOf(point) > 1) {
        surcharged++;
      }
    }
    return surcharged;
  }

  private static int turnsIn(List<String> route) {
    return turnsIn(map, route);
  }

  private static int turnsIn(Roadmap roadmap, List<String> route) {
    int turns = 0;
    for (int i = 2; i < route.size(); i++) {
      if (!sameHeading(roadmap, roadmap.idOf(route.get(i - 2)), roadmap.idOf(route.get(i - 1)),
          roadmap.idOf(route.get(i)))) {
        turns++;
      }
    }
    return turns;
  }

  private static boolean sameHeading(Roadmap roadmap, int first, int second, int third) {
    return Math.signum(roadmap.xOf(second) - roadmap.xOf(first)) == Math
        .signum(roadmap.xOf(third) - roadmap.xOf(second))
        && Math.signum(roadmap.yOf(second) - roadmap.yOf(first)) == Math
            .signum(roadmap.yOf(third) - roadmap.yOf(second));
  }

  private static int[] hopsTo(int goal) {
    int[] hops = new int[map.size()];
    Arrays.fill(hops, map.size());
    hops[goal] = 0;
    Deque<Integer> frontier = new ArrayDeque<>();
    frontier.add(goal);
    while (!frontier.isEmpty()) {
      int reached = frontier.poll();
      for (int predecessor : map.predecessorsOf(reached)) {
        if (hops[predecessor] == map.size()) {
          hops[predecessor] = hops[reached] + 1;
          frontier.add(predecessor);
        }
      }
    }
    return hops;
  }

  private static List<String> routeOf(Lacam2Solver.Result result, int agent) {
    return routeOf(map, result, agent);
  }

  private static List<String> routeOf(Roadmap roadmap, Lacam2Solver.Result result, int agent) {
    List<String> route = new ArrayList<>();
    for (int[] config : result.solution()) {
      String point = roadmap.nameOf(config[agent]);
      if (route.isEmpty() || !route.get(route.size() - 1).equals(point)) {
        route.add(point);
      }
    }
    return route;
  }

  private static List<String> violationsAfterMoving(Quality quality, int agent, int vertex) {
    List<int[]> plan = new ArrayList<>(quality.plan());
    int[] corrupted = plan.get(quality.makespan()).clone();
    corrupted[agent] = vertex;
    plan.set(quality.makespan(), corrupted);
    int agents = quality.instance().agents();
    return violationsIn(quality.instance(), plan, transpose(plan, agents), false);
  }

  private static long sumOfCosts(int[][] paths, int[] goals) {
    long total = 0;
    for (int agent = 0; agent < paths.length; agent++) {
      for (int t = paths[agent].length - 1; t >= 0; t--) {
        if (paths[agent][t] != goals[agent]) {
          total += t + 1;
          break;
        }
      }
    }
    return total;
  }

  private static long[] waitAndOffGoalTicks(int[][] paths, int[] goals) {
    long waits = 0;
    long offGoal = 0;
    for (int agent = 0; agent < paths.length; agent++) {
      for (int t = 0; t + 1 < paths[agent].length; t++) {
        if (paths[agent][t] == goals[agent]) {
          continue;
        }
        offGoal++;
        if (paths[agent][t + 1] == paths[agent][t]) {
          waits++;
        }
      }
    }
    return new long[] { waits, offGoal };
  }

  private static int[] backstepKinds(int[][] paths) {
    int[] tally = new int[2];
    for (int agent = 0; agent < paths.length; agent++) {
      int[] kinds = oscillationsOf(paths, agent);
      tally[0] += kinds[0];
      tally[1] += kinds[1];
    }
    return tally;
  }

  private static int[] oscillationsOf(int[][] paths, int agent) {
    int[] tally = new int[2];
    int[] path = paths[agent];
    int at = 0;
    while (true) {
      int left = nextChange(path, at);
      int returned = left < 0 ? -1 : nextChange(path, left);
      if (returned < 0) {
        return tally;
      }
      if (path[returned] != path[at]) {
        at = left;
        continue;
      }
      tally[visitedByOther(paths, agent, path[at], left, returned) ? 0 : 1]++;
      at = returned;
    }
  }

  private static int nextChange(int[] path, int from) {
    for (int step = from + 1; step < path.length; step++) {
      if (path[step] != path[from]) {
        return step;
      }
    }
    return -1;
  }

  private static boolean visitedByOther(
      int[][] paths, int agent, int vacated, int fromStep, int toStep) {
    for (int other = 0; other < paths.length; other++) {
      if (other == agent) {
        continue;
      }
      for (int step = fromStep; step < toStep && step < paths[other].length; step++) {
        if (paths[other][step] == vacated) {
          return true;
        }
      }
    }
    return false;
  }

  private static int[][] transpose(List<int[]> plan, int agents) {
    int[][] paths = new int[agents][plan.size()];
    for (int t = 0; t < plan.size(); t++) {
      for (int agent = 0; agent < agents; agent++) {
        paths[agent][t] = plan.get(t)[agent];
      }
    }
    return paths;
  }

  private static Instance instanceOn(int[] core, int agents, int seed) {
    int[] pool = core.clone();
    long state = seed * 0x9E3779B97F4A7C15L + 0x6A09E667F3BCC909L;
    for (int i = 0; i < 2 * agents; i++) {
      state = state * 6364136223846793005L + 1442695040888963407L;
      int pick = i + Math.floorMod(state >>> 33, pool.length - i);
      int swap = pool[i];
      pool[i] = pool[pick];
      pool[pick] = swap;
    }
    return new Instance(
        agents, seed,
        Arrays.copyOfRange(pool, 0, agents),
        Arrays.copyOfRange(pool, agents, 2 * agents));
  }

  private static int[] stronglyConnectedCore(Roadmap roadmap) {
    int anchor = busiestVertex(roadmap);
    int[] toAnchor = roadmap.distanceTo(anchor);
    boolean[] fromAnchor = reachableFrom(roadmap, anchor);
    List<Integer> core = new ArrayList<>();
    for (int vertex = 0; vertex < roadmap.size(); vertex++) {
      if (fromAnchor[vertex] && toAnchor[vertex] < Roadmap.INF) {
        core.add(vertex);
      }
    }
    int[] vertices = new int[core.size()];
    for (int i = 0; i < vertices.length; i++) {
      vertices[i] = core.get(i);
    }
    return vertices;
  }

  private static int busiestVertex(Roadmap roadmap) {
    int best = 0;
    for (int vertex = 1; vertex < roadmap.size(); vertex++) {
      if (roadmap.neighborsOf(vertex).length > roadmap.neighborsOf(best).length) {
        best = vertex;
      }
    }
    return best;
  }

  private static boolean[] reachableFrom(Roadmap roadmap, int source) {
    boolean[] seen = new boolean[roadmap.size()];
    Deque<Integer> queue = new ArrayDeque<>();
    seen[source] = true;
    queue.add(source);
    while (!queue.isEmpty()) {
      for (int neighbor : roadmap.neighborsOf(queue.poll())) {
        if (!seen[neighbor]) {
          seen[neighbor] = true;
          queue.add(neighbor);
        }
      }
    }
    return seen;
  }

  private static int maxOf(int[] values) {
    int max = values[0];
    for (int value : values) {
      max = Math.max(max, value);
    }
    return max;
  }

  private static String label(Quality quality) {
    return String.format(Locale.ROOT, "%s agents=%d seed=%d",
        modeOf(quality), quality.instance().agents(), quality.instance().seed());
  }

  private static String modeOf(Quality quality) {
    return quality.window() == SolverParams.WINDOW_FULL
        ? "full-horizon"
        : "window=" + quality.window();
  }

  private static String report() {
    StringBuilder out = new StringBuilder();
    out.append(String.format(Locale.ROOT,
        "%n[plan quality] map=%s points=%d core=%d loopLimit=%d window=%d%n",
        MAP_FILE, map.size(), stronglyConnectedCore(map).length, LOOP_LIMIT, WINDOW));
    out.append(String.format(Locale.ROOT,
        "%-13s %6s %5s %5s %8s %8s %8s %6s %6s %6s %8s%n",
        "mode", "agents", "seed", "span", "progress", "subopt", "wait", "yield", "wobble",
        "stall", "ms"));
    for (Quality quality : allPlans()) {
      out.append(row(quality));
    }
    return out.toString();
  }

  private static String row(Quality quality) {
    boolean unwindowed = quality.window() == SolverParams.WINDOW_FULL;
    return String.format(Locale.ROOT,
        "%-13s %6d %5d %5d %8.3f %8s %8.3f %6d %6d %6d %8d%n",
        modeOf(quality),
        quality.instance().agents(),
        quality.instance().seed(),
        quality.makespan(),
        quality.progressRate(),
        unwindowed ? String.format(Locale.ROOT, "%.3f", quality.suboptimality()) : "-",
        quality.waitRate(),
        quality.yieldingBacksteps(),
        quality.wobblingBacksteps(),
        quality.stalledAgents(),
        quality.runtimeMs());
  }

  record Instance(int agents, int seed, int[] starts, int[] goals) {
  }

  record Quality(
      Instance instance,
      int window,
      int makespan,
      List<int[]> plan,
      List<String> violations,
      boolean reachedGoals,
      long lowerBound,
      long sumOfCosts,
      long progressMade,
      long progressAvailable,
      int yieldingBacksteps,
      int wobblingBacksteps,
      long waitTicks,
      long offGoalTicks,
      int stalledAgents,
      long runtimeMs) {

    double suboptimality() {
      return sumOfCosts / (double) Math.max(1L, lowerBound);
    }

    double progressRate() {
      return progressMade / (double) Math.max(1L, progressAvailable);
    }

    double waitRate() {
      return waitTicks / (double) Math.max(1L, offGoalTicks);
    }

    double wobbleRate() {
      return wobblingBacksteps / (double) Math.max(1, instance.agents() * makespan);
    }

    double stalledShare() {
      return stalledAgents / (double) instance.agents();
    }
  }
}
