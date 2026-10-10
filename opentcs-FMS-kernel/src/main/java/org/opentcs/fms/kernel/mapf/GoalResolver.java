package org.opentcs.fms.kernel.mapf;

import java.util.List;
import java.util.Optional;
import org.opentcs.components.kernel.services.InternalTCSObjectService;
import org.opentcs.data.TCSObjectReference;
import org.opentcs.data.model.Location;
import org.opentcs.data.model.Point;
import org.opentcs.data.order.DriveOrder;

public final class GoalResolver {

  private static final org.slf4j.Logger logger =
      org.slf4j.LoggerFactory.getLogger(GoalResolver.class);

  private GoalResolver() {
  }

  public static List<String> goalCandidates(
      DriveOrder.Destination destination, InternalTCSObjectService objectService) {
    TCSObjectReference<?> ref = destination.getDestination();
    if (ref == null) {
      return List.of();
    }
    if (Point.class.equals(ref.getReferentClass())) {
      return List.of(ref.getName());
    }
    if (Location.class.equals(ref.getReferentClass())) {
      Optional<Location> location = objectService.fetch(Location.class, ref.getName());
      if (location.isPresent()) {
        String operation = destination.getOperation();
        return location.get().getAttachedLinks().stream()
            .filter(link -> link.getAllowedOperations().isEmpty()
                || link.getAllowedOperations().contains(operation))
            .map(link -> link.getPoint().getName())
            .sorted()
            .toList();
      }
    }
    return List.of();
  }

  public static Optional<String> goalPoint(
      DriveOrder.Destination destination, InternalTCSObjectService objectService) {
    List<String> candidates = goalCandidates(destination, objectService);
    if (candidates.size() > 1) {
      logger.warn(
          "Destination {} resolved to {} candidate points {}; using the first.",
          destination.getDestination().getName(), candidates.size(), candidates);
    }
    return candidates.stream().findFirst();
  }
}
