package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.pibt.PibtEngine;

class ExitPreferenceTest {

  private static final String MAP_FILE = "v7-vda5050-test.xml";

  private static final String STRANDED_ON = "3098";
  private static final String ONLY_DOOR = "3108";
  private static final String ALONG_THE_BLOCK = "3097";
  private static final String NORTH_OF_THE_DOOR = "3106";

  private static Roadmap map;

  @BeforeAll
  static void loadTheProductionMap() throws Exception {
    File file = RoadmapXmlLoader.findMap(MAP_FILE);
    assumeTrue(file != null, MAP_FILE + " not found; skipping");
    map = new Roadmap(RoadmapXmlLoader.load(file));
  }

  @Test
  void theBlockIsShapedSoThatCrawlingAlongItLooksCheaperThanWaiting() {
    for (String point : List.of(STRANDED_ON, ONLY_DOOR, ALONG_THE_BLOCK, NORTH_OF_THE_DOOR)) {
      assumeTrue(map.has(point), point + " is missing from " + MAP_FILE);
    }
    assertThat(map.isDropOffSlot(map.idOf(STRANDED_ON)))
        .as("%s must be a drop-off slot for the rule to apply at all", STRANDED_ON)
        .isTrue();
    assertThat(map.isCargoSlot(map.idOf(ALONG_THE_BLOCK)))
        .as("%s is the tempting sideways cell and must itself be inside the block",
            ALONG_THE_BLOCK)
        .isTrue();
    assertThat(map.isCargoSlot(map.idOf(ONLY_DOOR)))
        .as("%s is the aisle cell, the only way out of %s", ONLY_DOOR, STRANDED_ON)
        .isFalse();

    int[] distanceToTheGoal = map.distanceTo(map.idOf(NORTH_OF_THE_DOOR));
    assertThat(distanceToTheGoal[map.idOf(ALONG_THE_BLOCK)])
        .as("crawling to %s must look strictly cheaper than holding at %s, otherwise this"
            + " fixture cannot tell the tier apart from plain cost",
            ALONG_THE_BLOCK, STRANDED_ON)
        .isLessThan(distanceToTheGoal[map.idOf(STRANDED_ON)]);
  }

  @Test
  void aDroppingOffVehicleCanLeaveTheDropOffBlockThroughItsDoor() {
    assertThat(whereTheDroppingOffVehicleGoes())
        .as("a vehicle finishing drop-off must be allowed to leave through %s", ONLY_DOOR)
        .isEqualTo(ONLY_DOOR);
  }

  private static String whereTheDroppingOffVehicleGoes() {
    int stranded = map.idOf(STRANDED_ON);
    int door = map.idOf(ONLY_DOOR);
    int[] goals = { door };

    int[] next = new PibtEngine(
        map, 1, 1, DistTable.weighted(map, goals), null, null,
        new LegalMoves(map, goals)).step(
        new int[] { stranded },
        new int[] { 0 },
        List.of(),
        null,
        null,
        StaticBodies.NONE,
        null);
    return map.nameOf(next[0]);
  }

}
