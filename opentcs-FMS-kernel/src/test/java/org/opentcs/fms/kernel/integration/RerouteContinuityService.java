package org.opentcs.fms.kernel.integration;

import java.util.List;

final class RerouteContinuityService {

  private RerouteContinuityService() {
  }

  static boolean canApply(List<String> sentNotYetExecuted, List<String> newRoute) {
    if (sentNotYetExecuted.size() < 2) {
      return true;
    }
    if (newRoute.size() < sentNotYetExecuted.size()) {
      return false;
    }
    return newRoute.subList(0, sentNotYetExecuted.size()).equals(sentNotYetExecuted);
  }
}
