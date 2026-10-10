package org.opentcs.fms.kernel.mapf;

import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.DistTable;
import org.opentcs.fms.routing.FmsLocalGuide;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.RoadmapXmlLoader;
import org.opentcs.fms.routing.pibt.Footprint;

class SeedAnchorTraceTest {

  @Test
  void reportsHowFarAVehicleMayTravelBeforeItsCarriedGuidePathStopsAnchoring() throws Exception {
    File file = RoadmapXmlLoader.findMap("v7-vda5050-test.xml");
    assumeTrue(file != null, "production map not found; skipping");
    Roadmap map = new Roadmap(RoadmapXmlLoader.load(file));
    System.setProperty("fms.guide.on", "true");
    System.setProperty("fms.guide.deterministic", "true");
    System.setProperty("fms.guide.numRefine", "2");

    List<String> lane = List.of("0085", "0086", "0087", "0105", "0115", "0125", "0135", "0145");
    int[] plan = new int[lane.size()];
    for (int i = 0; i < plan.length; i++) {
      plan[i] = map.idOf(lane.get(i));
    }
    int[] goals = { map.idOf("0145") };
    DistTable dist = DistTable.turnAware(map, goals);

    System.out.println("[anchor] plan = " + lane);
    for (int travelled = 0; travelled <= 4; travelled++) {
      int start = plan[travelled];
      int[] seed = MapfPlanner.planFromWhereTheVehicleActuallyIs(plan, start);
      FmsLocalGuide guide = FmsLocalGuide.builder(
          map,
          dist,
          goals,
          0
      )
          .footprint(new Footprint(map))
          .loaded(new boolean[] { true })
          .build();
      guide.setGuidePaths(new int[][] { seed });
      guide.construct(new int[] { start }, null, new int[] { 0 });

      List<String> head = new ArrayList<>();
      for (int i = 0; i < Math.min(3, seed.length); i++) {
        head.add(seed[i] < 0 ? "-" : map.nameOf(seed[i]));
      }
      System.out.printf(
          "[anchor] di %d o, dung tai %s | seed dau=%s | seed[1] la vi tri that? %-5s"
              + " | warmStartSlides=%d%n",
          travelled, map.nameOf(start), head,
          seed.length > 1 && seed[1] == start, guide.counters().warmStartSlides);
    }
  }
}
