package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.File;
import java.util.List;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.pibt.Footprint;

/** Characterizes the loaded-vehicle footprint deadlock observed on 2026-09-09. */
class FootprintDeadlockRegressionTest {

  private static final String MAP_FILE = "v7-Xuonghientai.xml";
  private static final int WINDOW = 10;

  private static String savedFootprint;
  private static String savedClearance;
  private static String savedGuide;
  private static String savedGuideProfile;

  @BeforeAll
  static void useTheProductionFootprintAndGuideConfiguration() {
    savedFootprint = System.getProperty("fms.pibt.footprint");
    savedClearance = System.getProperty("fms.pibt.clearanceMm");
    savedGuide = System.getProperty("fms.guide.on");
    savedGuideProfile = System.getProperty("fms.guide.profile");
    System.setProperty("fms.pibt.footprint", "true");
    System.setProperty("fms.pibt.clearanceMm", "0");
    System.setProperty("fms.guide.on", "true");
    System.setProperty("fms.guide.profile", "fms");
  }

  @AfterAll
  static void restoreProperties() {
    restore("fms.pibt.footprint", savedFootprint);
    restore("fms.pibt.clearanceMm", savedClearance);
    restore("fms.guide.on", savedGuide);
    restore("fms.guide.profile", savedGuideProfile);
  }

  @Test
  void windowedSolverAdmitsTheV2V6ClosedFootprintDeadlock() throws Exception {
    Roadmap map = loadProductionMap();
    int v2 = 1;
    int v6 = 5;

    int[][] carriedGuides = paths(map,
        new String[] {
            "0021", "0352", "0324", "0006", "0001", "0531", "0532", "0386",
            "0387", "0535", "0536", "0538", "0539", "0540", "0541", "0542",
            "0543", "0544", "0545", "0546", "0548", "0550", "0551", "0552",
            "0124", "0134", "0144", "0154", "0164", "0174"
        },
        new String[] {
            "0548", "0550", "0551", "0552", "0553", "0554", "0555", "0127", "0137"
        },
        new String[] {
            "0543", "0544", "0545", "0546", "0548", "0550", "0551", "0552",
            "0553", "0554", "0555", "0556", "0128"
        },
        new String[] {
            "0407", "0406", "0405", "0404", "0045", "0326", "0325", "0324",
            "0323", "0351", "0022", "0303", "0308"
        },
        new String[] {
            "0477", "0594", "0593", "0592", "0591", "0590", "0589", "0588",
            "0586", "0501", "0499", "0498", "0065", "0064", "0063", "0062",
            "0061", "0060", "0047", "0045", "0005", "0325", "0353", "0020",
            "0301", "0306", "0311"
        },
        new String[] {
            "0552", "0553", "0554", "0555", "0127", "0137", "0147", "0157",
            "0167", "0177"
        },
        new String[] {
            "0305", "0024", "0349", "0317", "0009", "0004", "0448", "0449",
            "0385", "0531", "0532", "0386", "0387", "0535", "0536", "0538",
            "0539", "0540", "0541", "0542", "0543", "0544", "0545", "0546",
            "0548", "0550", "0551", "0552", "0124", "0134"
        });

    int[] startAxis = enteringDirections(map, carriedGuides);

    Lacam2Solver.Result result = new Lacam2Solver(map).solve(
        new MapfInstance(
            ids(map, "0352", "0550", "0544", "0406", "0594", "0553", "0024"),
            ids(map, "0174", "0137", "0128", "0308", "0311", "0177", "0134")),
        5_000L,
        0,
        Lacam2Solver.Objective.NONE,
        Lacam2Solver.DEFAULT_RESTART_RATE,
        200_000,
        WINDOW,
        new AgentPriority[] {
            new AgentPriority(0, 0, 48, 0, 0f),
            new AgentPriority(0, 0, 44, 0, 0f),
            new AgentPriority(0, 0, 35, 0, 0f),
            new AgentPriority(0, 0, 42, 0, 0f),
            new AgentPriority(0, 0, 41, 0, 0f),
            new AgentPriority(0, 0, 38, 0, 0f),
            new AgentPriority(0, 0, 35, 0, 0f)
        },
        startAxis,
        new boolean[] {true, true, true, false, false, true, true},
        carriedGuides,
        null,
        new StaticBodies(
            ids(map, "0304"),
            new int[] {map.directionOf(map.idOf("0023"), map.idOf("0304"))},
            new boolean[] {true}),
        startAxis);

    assertThat(result.solved()).isTrue();
    assertThat(result.reachedGoals()).isFalse();

    int deadlockedAt = indexOf(
        result.solution(), v2, map.idOf("0554"), v6, map.idOf("0555"));
    assertThat(deadlockedAt)
        .as("the current one-step model lets V2 enter 0554 beside V6 at the 0555 turn")
        .isEqualTo(4);
    assertThat(result.solution().subList(deadlockedAt, result.solution().size()))
        .as("once admitted, the closed pair is represented as collision-free waits to the window")
        .allSatisfy(config -> {
          assertThat(config[v2]).isEqualTo(map.idOf("0554"));
          assertThat(config[v6]).isEqualTo(map.idOf("0555"));
        });

    Footprint footprint = new Footprint(map);
    assertThat(footprint.collideMoving(
        Footprint.Motion.resting(map.idOf("0554"), Roadmap.DIR_NEG_X, true),
        new Footprint.Motion(
            map.idOf("0555"), map.idOf("0127"), Roadmap.DIR_NEG_X, true)))
        .as("V2 blocks the loaded swept footprint V6 needs to turn from 0555 to 0127")
        .isTrue();
  }

  private static Roadmap loadProductionMap() throws Exception {
    File file = RoadmapXmlLoader.findMap(MAP_FILE);
    assertThat(file).as("production test map %s", MAP_FILE).isNotNull();
    return new Roadmap(RoadmapXmlLoader.load(file));
  }

  private static int[] ids(Roadmap map, String... names) {
    int[] ids = new int[names.length];
    for (int i = 0; i < names.length; i++) {
      ids[i] = map.idOf(names[i]);
    }
    return ids;
  }

  private static int[][] paths(Roadmap map, String[]... paths) {
    int[][] result = new int[paths.length][];
    for (int i = 0; i < paths.length; i++) {
      result[i] = ids(map, paths[i]);
    }
    return result;
  }

  private static int[] enteringDirections(Roadmap map, int[][] paths) {
    int[] result = new int[paths.length];
    for (int i = 0; i < paths.length; i++) {
      result[i] = map.directionOf(paths[i][0], paths[i][1]);
    }
    return result;
  }

  private static int indexOf(
      List<int[]> plan, int firstAgent, int firstPoint, int secondAgent, int secondPoint) {
    for (int tick = 0; tick < plan.size(); tick++) {
      int[] config = plan.get(tick);
      if (config[firstAgent] == firstPoint && config[secondAgent] == secondPoint) {
        return tick;
      }
    }
    return -1;
  }

  private static void restore(String key, String value) {
    if (value == null) {
      System.clearProperty(key);
    }
    else {
      System.setProperty(key, value);
    }
  }
}
