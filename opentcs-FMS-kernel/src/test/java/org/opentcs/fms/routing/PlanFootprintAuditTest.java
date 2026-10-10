package org.opentcs.fms.routing;

import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.pibt.Footprint;

class PlanFootprintAuditTest {

  private static final String PLANS_PROPERTY = "fms.audit.plans";
  private static final boolean LOADED = Boolean.parseBoolean(System.getProperty("fms.audit.loaded", "true"));

  private static final String MAP_FILE = "v7-vda5050-test.xml";

  @Test
  void auditsRecordedPlansAgainstTheFootprintPibtEnforces() throws Exception {
    String plansPath = System.getProperty(PLANS_PROPERTY);
    assumeTrue(plansPath != null, PLANS_PROPERTY + " not set; skipping");
    File mapFile = RoadmapXmlLoader.findMap(MAP_FILE);
    assumeTrue(mapFile != null, MAP_FILE + " not found; skipping");

    Roadmap map = new Roadmap(RoadmapXmlLoader.load(mapFile));
    Footprint footprint = new Footprint(map);

    Map<Integer, Map<String, int[]>> solves = new LinkedHashMap<>();
    int unknownPoints = 0;
    for (String line : Files.readAllLines(Path.of(plansPath), StandardCharsets.UTF_8)) {
      String[] parts = line.split("\t");
      if (parts.length != 3) {
        continue;
      }
      String[] names = parts[2].trim().split("\s+");
      int[] path = new int[names.length];
      boolean usable = true;
      for (int t = 0; t < names.length; t++) {
        if (!map.has(names[t])) {
          usable = false;
          unknownPoints++;
          break;
        }
        path[t] = map.idOf(names[t]);
      }
      if (usable) {
        solves.computeIfAbsent(Integer.parseInt(parts[0]), k -> new LinkedHashMap<>())
            .put(parts[1], path);
      }
    }

    int sameCell = 0;
    int overlaps = 0;
    int pairTicks = 0;
    List<String> samples = new ArrayList<>();

    for (Map.Entry<Integer, Map<String, int[]>> solve : solves.entrySet()) {
      List<String> names = new ArrayList<>(solve.getValue().keySet());
      for (int i = 0; i < names.size(); i++) {
        for (int j = i + 1; j < names.size(); j++) {
          int[] a = solve.getValue().get(names.get(i));
          int[] b = solve.getValue().get(names.get(j));
          int ticks = Math.min(a.length, b.length) - 1;
          int noseA = Roadmap.DIR_NONE;
          int noseB = Roadmap.DIR_NONE;
          for (int t = 0; t < ticks; t++) {
            Footprint.Motion ma = new Footprint.Motion(a[t], a[t + 1], noseA, LOADED);
            Footprint.Motion mb = new Footprint.Motion(b[t], b[t + 1], noseB, LOADED);
            if (footprint.mayInteract(a[t], b[t])) {
              pairTicks++;
              if (a[t] == b[t] || a[t + 1] == b[t + 1]) {
                sameCell++;
              } else if (footprint.collideMoving(ma, mb)) {
                overlaps++;
                if (samples.size() < 8) {
                  samples.add(String.format(
                      "solve=%d t=%d %s %s->%s x %s %s->%s",
                      solve.getKey(), t, names.get(i), map.nameOf(a[t]), map.nameOf(a[t + 1]),
                      names.get(j), map.nameOf(b[t]), map.nameOf(b[t + 1])));
                }
              }
            }
            noseA = footprint.noseAfter(noseA, a[t], a[t + 1]);
            noseB = footprint.noseAfter(noseB, b[t], b[t + 1]);
          }
        }
      }
    }

    System.out.printf(
        "[plan-audit] solves=%d unknownPoints=%d pairTicksWithinReach=%d"
            + " sameCell=%d footprintOverlaps=%d%n",
        solves.size(), unknownPoints, pairTicks, sameCell, overlaps);
    for (String sample : samples) {
      System.out.printf("    %s%n", sample);
    }
  }
}
