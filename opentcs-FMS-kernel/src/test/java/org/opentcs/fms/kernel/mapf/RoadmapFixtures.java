package org.opentcs.fms.kernel.mapf;

import java.util.ArrayList;
import java.util.List;
import org.opentcs.fms.routing.Roadmap;

final class RoadmapFixtures {

  private RoadmapFixtures() {
  }

  static Roadmap gridRoadmap(int width, int height) {
    List<Roadmap.Point> points = new ArrayList<>();
    for (int row = 0; row < height; row++) {
      for (int col = 0; col < width; col++) {
        points.add(new Roadmap.Point("r" + row + "c" + col, col, row, 0));
      }
    }
    List<Roadmap.Path> paths = new ArrayList<>();
    for (int row = 0; row < height; row++) {
      for (int col = 0; col < width; col++) {
        if (col + 1 < width) {
          paths.add(new Roadmap.Path("e", "r" + row + "c" + col, "r" + row + "c" + (col + 1), 1, true));
        }
        if (row + 1 < height) {
          paths.add(new Roadmap.Path("e", "r" + row + "c" + col, "r" + (row + 1) + "c" + col, 1, true));
        }
      }
    }
    return new Roadmap(new Roadmap.Model(points, paths));
  }

  static Roadmap columnRoadmap(String... pointNames) {
    List<Roadmap.Point> points = new ArrayList<>();
    List<Roadmap.Path> paths = new ArrayList<>();
    for (int i = 0; i < pointNames.length; i++) {
      points.add(new Roadmap.Point(pointNames[i], 0, pointNames.length - 1 - i, 0));
      if (i + 1 < pointNames.length) {
        paths.add(bidirectionalEdge(pointNames[i], pointNames[i + 1]));
      }
    }
    return new Roadmap(new Roadmap.Model(points, paths));
  }

  static Roadmap roadmapOf(List<String> pointNames, List<Roadmap.Path> paths) {
    List<Roadmap.Point> points = new ArrayList<>();
    for (String name : pointNames) {
      points.add(new Roadmap.Point(name, 0, 0, 0));
    }
    return new Roadmap(new Roadmap.Model(points, paths));
  }

  static Roadmap.Path edge(String src, String dst) {
    return new Roadmap.Path(src + "->" + dst, src, dst, 1, false);
  }

  static Roadmap.Path bidirectionalEdge(String src, String dst) {
    return new Roadmap.Path(src + "<->" + dst, src, dst, 1, true);
  }

  static int[] startsAt(Roadmap r, String... pointNames) {
    int[] starts = new int[pointNames.length];
    for (int i = 0; i < pointNames.length; i++) {
      starts[i] = r.idOf(pointNames[i]);
    }
    return starts;
  }
}
