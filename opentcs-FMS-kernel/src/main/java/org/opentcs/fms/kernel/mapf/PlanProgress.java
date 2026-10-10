package org.opentcs.fms.kernel.mapf;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

record PlanProgress(List<String> route, int step) {

  static Optional<PlanProgress> of(List<String> planRoute, List<String> stillToAllocate) {
    if (planRoute.isEmpty()) {
      return Optional.empty();
    }
    if (stillToAllocate.isEmpty()) {
      return Optional.of(new PlanProgress(planRoute, planRoute.size() - 1));
    }
    int sharedLength = lengthWhereRouteEndMeetsClaimStart(planRoute, stillToAllocate);
    if (sharedLength == 0) {
      return Optional.empty();
    }
    return Optional.of(new PlanProgress(planRoute, planRoute.size() - sharedLength));
  }

  String pointUnderRequest() {
    return route.get(step);
  }

  boolean hasAlreadyPassedAnyOf(Collection<String> points) {
    List<String> passed = route.subList(0, step);
    List<String> stillAhead = route.subList(step, route.size());
    return points.stream().anyMatch(point -> passed.contains(point) && !stillAhead.contains(point));
  }

  private static int lengthWhereRouteEndMeetsClaimStart(
      List<String> planRoute, List<String> stillToAllocate) {
    for (int length = Math.min(planRoute.size(), stillToAllocate.size()); length > 0; length--) {
      List<String> routeEnd = planRoute.subList(planRoute.size() - length, planRoute.size());
      List<String> claimStart = stillToAllocate.subList(0, length);
      if (routeEnd.equals(claimStart)) {
        return length;
      }
    }
    return 0;
  }
}
