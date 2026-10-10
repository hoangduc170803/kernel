package org.opentcs.fms.kernel.mapf;

import org.opentcs.fms.routing.Roadmap;

final class RouteShapeLogger {

  private static final org.slf4j.Logger logger =
      org.slf4j.LoggerFactory.getLogger(RouteShapeLogger.class);

  private static final int GUIDE_PREVIEW_LENGTH = 12;

  private final Roadmap roadmap;

  RouteShapeLogger(Roadmap roadmap) {
    this.roadmap = roadmap;
  }

  void logAnomalies(String vehicle, int[] raw, int[] simple, int goal, int[] guidePath) {
    if (!logger.isInfoEnabled()) {
      return;
    }
    int waitsDropped = 0;
    StringBuilder waitCells = new StringBuilder();
    for (int t = 1; t < raw.length; t++) {
      if (raw[t] == raw[t - 1] && raw[t] != goal) {
        waitsDropped++;
        waitCells.append(waitCells.length() == 0 ? "" : " ")
            .append(roadmap.nameOf(raw[t])).append("@t").append(t);
      }
    }
    String revisited = firstRevisit(simple);
    if (waitsDropped == 0 && revisited == null) {
      return;
    }
    logger.info(
        "[MAPF route-shape] {} raw={} installed={} waitsDropped={} at [{}]"
            + " revisits={} | guide said [{}]",
        vehicle, raw.length, simple.length, waitsDropped, waitCells,
        revisited == null ? "no" : revisited, guidePreview(guidePath));
  }

  private String firstRevisit(int[] simple) {
    for (int i = 1; i < simple.length; i++) {
      for (int j = 0; j < i - 1; j++) {
        if (simple[j] == simple[i]) {
          return roadmap.nameOf(simple[i]) + "(index " + j + " then " + i + ")";
        }
      }
    }
    return null;
  }

  private StringBuilder guidePreview(int[] guidePath) {
    StringBuilder guided = new StringBuilder();
    if (guidePath == null) {
      return guided;
    }
    for (int t = 0; t < Math.min(GUIDE_PREVIEW_LENGTH, guidePath.length); t++) {
      guided.append(t == 0 ? "" : ", ")
          .append(guidePath[t] < 0 ? "-" : roadmap.nameOf(guidePath[t]));
    }
    return guided;
  }
}
