import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.AgentPriority;
import org.opentcs.fms.routing.CollisionTable;
import org.opentcs.fms.routing.Lacam2Solver;
import org.opentcs.fms.routing.MapfInstance;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.RoadmapXmlLoader;
import org.opentcs.fms.routing.SolverParams;
import org.opentcs.fms.routing.StaticBodies;
import org.opentcs.fms.routing.pibt.Footprint;

/**
 * Characterises the pre-reservation behaviour at the 0087/0088/0007 junction.
 *
 * <p>
 * Agent A approaches 0088 from above with higher priority and is bound for
 * 0004. Agent B is
 * loaded, points east at 0087, and is bound for 0091. The useful choice is
 * between pivoting early
 * at 0088 (which sweeps B) and continuing south to pivot at 0007 (which lets B
 * wait).
 * </p>
 */
class TurningYieldPlanTest {

  private static final String MAP_FILE = "v7-vda5050-test.xml";
  private static final String FOOTPRINT_PROPERTY = "fms.pibt.footprint";
  private static final String CLEARANCE_PROPERTY = "fms.pibt.clearanceMm";
  private static final String GUIDE_PROPERTY = "fms.guide.on";
  private static final String GUIDE_DETERMINISTIC_PROPERTY = "fms.guide.deterministic";
  private static final String GUIDE_REFINE_PROPERTY = "fms.guide.numRefine";
  private static final String GUIDE_RESERVE_FOOTPRINT_PROPERTY = "fms.guide.reserveFootprint";

  private static final int A = 0;
  private static final int B = 1;
  private static final int SEED = 0;

  private static Roadmap map;

  @BeforeAll
  static void loadMap() throws Exception {
    File file = RoadmapXmlLoader.findMap(MAP_FILE);
    assumeTrue(file != null, MAP_FILE + " not found; skipping");
    map = new Roadmap(RoadmapXmlLoader.load(file));
  }

  @AfterEach
  void restoreProperties() {
    System.clearProperty(FOOTPRINT_PROPERTY);
    System.clearProperty(CLEARANCE_PROPERTY);
    System.clearProperty(GUIDE_PROPERTY);
    System.clearProperty(GUIDE_DETERMINISTIC_PROPERTY);
    System.clearProperty(GUIDE_REFINE_PROPERTY);
    System.clearProperty(GUIDE_RESERVE_FOOTPRINT_PROPERTY);
  }

  @Test
  void reportsWhereTheBaselineGuideAndLacamPibtChooseToTurn() {
    System.setProperty(FOOTPRINT_PROPERTY, "true");
    System.setProperty(CLEARANCE_PROPERTY, "50");
    System.setProperty(GUIDE_PROPERTY, "true");
    System.setProperty(GUIDE_DETERMINISTIC_PROPERTY, "true");
    System.setProperty(GUIDE_REFINE_PROPERTY, "2");
    System.setProperty(GUIDE_RESERVE_FOOTPRINT_PROPERTY, "true");

    int[] starts = ids("0106", "0087");
    int[] goals = ids("0004", "0091");
    int[] noses = { Roadmap.DIR_NEG_Y, Roadmap.DIR_POS_X };
    boolean[] loaded = { true, true };
    AgentPriority[] priorities = {
        new AgentPriority(0, 0, 10, 0, 0.0f),
        AgentPriority.NEWCOMER
    };

    Footprint footprint = new Footprint(map);
    assertThat(footprint.maxReach() - 2 * footprint.sweptRadius(true))
        .as("the test must exercise the shipped 50 mm clearance")
        .isEqualTo(50.0);
    assertThat(footprint.collideMoving(
        motion("0088", "0089", Roadmap.DIR_NEG_Y),
        resting("0087", Roadmap.DIR_POS_X)))
        .as("a loaded A pivoting early at 0088 must sweep loaded B at 0087")
        .isTrue();
    assertThat(footprint.collideMoving(
        motion("0007", "0006", Roadmap.DIR_NEG_Y),
        resting("0087", Roadmap.DIR_POS_X)))
        .as("the same loaded B must clear A's later pivot at 0007")
        .isFalse();

    Lacam2Solver.Result result = new Lacam2Solver(map).solve(
        new MapfInstance(starts, goals),
        5_000L,
        SEED,
        Lacam2Solver.Objective.NONE,
        Lacam2Solver.DEFAULT_RESTART_RATE,
        Lacam2Solver.LOOP_LIMIT_UNBOUNDED,
        SolverParams.WINDOW_FULL,
        priorities,
        noses,
        loaded,
        null,
        null,
        StaticBodies.NONE);

    assertThat(result.solved()).isTrue();
    assertThat(result.reachedGoals()).isTrue();
    assertThat(result.executedPriorities()[A].aged())
        .as("A must really be processed ahead of B at the root")
        .isGreaterThan(result.executedPriorities()[B].aged());
    assertThat(result.guidePaths()).isNotNull();
    assertNoFootprintCollision(result.solution(), noses, loaded, footprint);

    List<String> guideA = names(result.guidePaths()[A]);
    List<String> guideB = names(result.guidePaths()[B]);
    List<String> solvedA = namesOfAgent(result.solution(), A);
    List<String> solvedB = namesOfAgent(result.solution(), B);

    assertNoFootprintCollision(
        configurationsOf(result.guidePaths()), noses, loaded, footprint);
    assertThat(firstPivot(result.guidePaths()[A], noses[A], footprint))
        .as("the higher-order agent must continue south and pivot at the safe lower row")
        .startsWith("0007@");
    assertThat(waitsBefore(guideB, "0091"))
        .as("the lower-order agent should wait outside A's reserved turn footprint")
        .isPositive();
    assertThat(retreatsFrom0087(guideB))
        .as("waiting is enough in this geometry, so B must not be routed backwards")
        .isZero();
    assertThat(result.constrainedCalls())
        .as("a footprint-safe guide should not need LaCAM constraints to repair this case")
        .isZero();

    System.out.printf(Locale.ROOT,
        "%n[turn-yield/reserved] clearance=50mm loaded=[true,true] order=A>B%n"
            + "  A: 0106(-Y) -> 0004%n"
            + "  B: 0087(+X) -> 0091%n"
            + "  local-guide A: %s%n"
            + "  local-guide B: %s%n"
            + "  lacam-pibt  A: %s%n"
            + "  lacam-pibt  B: %s%n"
            + "  guide first pivot A: %s%n"
            + "  solved first pivot A: %s%n"
            + "  solved B waits-before-goal=%d retreats=%d%n"
            + "  loops=%d constrained=%d nodes=%d runtime=%dms%n",
        guideA,
        guideB,
        solvedA,
        solvedB,
        firstPivot(result.guidePaths()[A], noses[A], footprint),
        firstPivot(verticesOfAgent(result.solution(), A), noses[A], footprint),
        waitsBefore(solvedB, "0091"),
        retreatsFrom0087(solvedB),
        result.loopCount(),
        result.constrainedCalls(),
        result.nodesGenerated(),
        result.runtimeMs());
  }

  private static int[] ids(String... names) {
    int[] vertices = new int[names.length];
    for (int i = 0; i < names.length; i++) {
      assertThat(map.has(names[i])).as("%s missing from %s", names[i], MAP_FILE).isTrue();
      vertices[i] = map.idOf(names[i]);
    }
    return vertices;
  }

  private static Footprint.Motion motion(String from, String to, int nose) {
    return new Footprint.Motion(map.idOf(from), map.idOf(to), nose, true);
  }

  private static Footprint.Motion resting(String at, int nose) {
    return Footprint.Motion.resting(map.idOf(at), nose, true);
  }

  private static void assertNoFootprintCollision(
      List<int[]> plan, int[] startNoses, boolean[] loaded, Footprint footprint) {
    int[] nose = startNoses.clone();
    for (int tick = 0; tick + 1 < plan.size(); tick++) {
      Footprint.Motion a = new Footprint.Motion(
          plan.get(tick)[A], plan.get(tick + 1)[A], nose[A], loaded[A]);
      Footprint.Motion b = new Footprint.Motion(
          plan.get(tick)[B], plan.get(tick + 1)[B], nose[B], loaded[B]);
      assertThat(footprint.collideMoving(a, b))
          .as("emitted motions clash at t=%d: A=%s->%s B=%s->%s",
              tick, map.nameOf(a.from()), map.nameOf(a.to()),
              map.nameOf(b.from()), map.nameOf(b.to()))
          .isFalse();
      nose[A] = footprint.noseAfter(nose[A], a.from(), a.to());
      nose[B] = footprint.noseAfter(nose[B], b.from(), b.to());
    }
  }

  private static List<String> names(int[] path) {
    List<String> names = new ArrayList<>();
    for (int vertex : path) {
      if (vertex == CollisionTable.NO_VERTEX) {
        break;
      }
      names.add(map.nameOf(vertex));
    }
    return names;
  }

  private static List<String> namesOfAgent(List<int[]> plan, int agent) {
    List<String> names = new ArrayList<>(plan.size());
    for (int[] configuration : plan) {
      names.add(map.nameOf(configuration[agent]));
    }
    return names;
  }

  private static int[] verticesOfAgent(List<int[]> plan, int agent) {
    int[] vertices = new int[plan.size()];
    for (int tick = 0; tick < plan.size(); tick++) {
      vertices[tick] = plan.get(tick)[agent];
    }
    return vertices;
  }

  private static List<int[]> configurationsOf(int[][] paths) {
    List<int[]> configurations = new ArrayList<>(paths[0].length);
    for (int tick = 0; tick < paths[0].length; tick++) {
      int[] configuration = new int[paths.length];
      for (int agent = 0; agent < paths.length; agent++) {
        configuration[agent] = paths[agent][tick];
      }
      configurations.add(configuration);
    }
    return configurations;
  }

  private static String firstPivot(int[] path, int startNose, Footprint footprint) {
    int nose = startNose;
    for (int tick = 0; tick + 1 < path.length; tick++) {
      if (path[tick] == CollisionTable.NO_VERTEX || path[tick + 1] == CollisionTable.NO_VERTEX) {
        break;
      }
      int after = footprint.noseAfter(nose, path[tick], path[tick + 1]);
      if (after != nose) {
        return map.nameOf(path[tick]) + "@t" + tick;
      }
      nose = after;
    }
    return "none";
  }

  private static int waitsBefore(List<String> path, String goal) {
    int waits = 0;
    for (int tick = 0; tick + 1 < path.size(); tick++) {
      if (path.get(tick).equals(goal)) {
        break;
      }
      if (path.get(tick).equals(path.get(tick + 1))) {
        waits++;
      }
    }
    return waits;
  }

  private static int retreatsFrom0087(List<String> path) {
    int retreats = 0;
    for (int tick = 0; tick + 1 < path.size(); tick++) {
      if (map.xOf(map.idOf(path.get(tick + 1))) < map.xOf(map.idOf(path.get(tick)))) {
        retreats++;
      }
    }
    return retreats;
  }
}
