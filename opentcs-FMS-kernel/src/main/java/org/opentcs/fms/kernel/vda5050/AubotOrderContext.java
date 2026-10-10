package org.opentcs.fms.kernel.vda5050;

import static java.util.Objects.requireNonNull;

import java.security.SecureRandom;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicLong;
import javax.annotation.Nullable;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.Route;
import org.opentcs.drivers.vehicle.MovementCommand;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class AubotOrderContext {

  static final int MAX_ACTION_ID_LENGTH = 10;
  static final int MAX_ORDER_ID_LENGTH = 10;
  private static final int ORDER_TOKEN_SEED_BOUND = 1 << 20;
  private static final char DRIVE_ORDER_INDEX_SEPARATOR = '-';
  private static final int EDGE_ID_AFFIX_LENGTH = 4;
  private static final int COMPACT_EDGE_ID_LENGTH = EDGE_ID_AFFIX_LENGTH * 2;
  private static final String DESTINATION_ACTION_ID_PREFIX = "Order_destination_action_";
  private static final String CUSTOM_ACTION_ID_PREFIX = "Order_destination_custom_action_";
  private static final String COMPACT_DESTINATION_ACTION_ID_PREFIX = "ODA_";
  private static final String COMPACT_CUSTOM_ACTION_ID_PREFIX = "ODC_";
  private static final String RESOURCE_ACTION_INFIX = "_action_";

  private static final Logger LOG = LoggerFactory.getLogger(AubotOrderContext.class);

  private final Map<String, String> directionByEdgeId = new ConcurrentHashMap<>();
  private final Map<String, Double> maxSpeedByEdgeId = new ConcurrentHashMap<>();
  private final Map<String, String> edgeIdByCompactEdgeId = new ConcurrentHashMap<>();
  private final Map<String, String> actionIdByCompactActionId = new ConcurrentHashMap<>();
  private final Map<String, String> orderIdByCompactOrderId = new ConcurrentHashMap<>();
  private final Map<String, String> orderTokenByOrderName = new ConcurrentHashMap<>();
  private final AtomicLong orderTokenSequence
      = new AtomicLong(new SecureRandom().nextInt(ORDER_TOKEN_SEED_BOUND));

  private final AubotSpeeds speeds;
  private final AubotDirection layoutDirection;

  public AubotOrderContext(AubotSpeeds speeds) {
    this(speeds, null);
  }

  public AubotOrderContext(
      AubotSpeeds speeds,
      @Nullable
      AubotDirection layoutDirection) {
    this.speeds = requireNonNull(speeds, "speeds");
    this.layoutDirection = layoutDirection;
  }

  public void record(MovementCommand command, boolean loaded) {
    requireNonNull(command, "command");

    List<Route.Step> steps = command.getDriveOrder().getRoute().getSteps();
    double cruisingSpeed = speeds.cruising(loaded);

    for (int index = 0; index < steps.size(); index++) {
      Route.Step step = steps.get(index);
      if (step.getPath() == null || step.getSourcePoint() == null) {
        continue;
      }
      boolean backward = drivesInReverse(step);
      String traversal = traversalKey(
          step.getSourcePoint().getName(), step.getDestinationPoint().getName());
      AubotDirection heading = headingOf(step);

      if (heading != null) {
        directionByEdgeId.put(traversal, heading.alignTo(layoutDirection).getCode());
      }

      double speed = turnsAfter(steps, index, heading) ? speeds.turning() : cruisingSpeed;
      maxSpeedByEdgeId.put(traversal, backward ? -speed : speed);
    }
  }

  private static boolean turnsAfter(
      List<Route.Step> steps,
      int index,
      @Nullable
      AubotDirection heading) {
    if (heading == null) {
      return false;
    }

    for (int next = index + 1; next < steps.size(); next++) {
      AubotDirection nextHeading = headingOf(steps.get(next));
      if (nextHeading != null) {
        return nextHeading != heading;
      }
    }
    return false;
  }

  @Nullable
  private static AubotDirection headingOf(Route.Step step) {
    return step.getSourcePoint() == null
        ? null
        : AubotDirection.between(
            step.getSourcePoint(),
            step.getDestinationPoint(),
            drivesInReverse(step));
  }

  public AubotDirection.Coordinates toVehicleCoordinates(double mapX, double mapY) {
    return layoutDirection == null
        ? new AubotDirection.Coordinates(mapX, mapY)
        : layoutDirection.toVehicleFrame(mapX, mapY);
  }

  public AubotDirection.Coordinates toMapCoordinates(double vehicleX, double vehicleY) {
    return layoutDirection == null
        ? new AubotDirection.Coordinates(vehicleX, vehicleY)
        : layoutDirection.toMapFrame(vehicleX, vehicleY);
  }

  public double mapAngleOf(double reportedTheta) {
    double layoutOffset = layoutDirection == null
        ? 0.0
        : layoutDirection.ordinal() * (Math.PI / 2);

    return Math.IEEEremainder(Math.PI / 2 - (reportedTheta + layoutOffset), 2 * Math.PI);
  }

  private static boolean drivesInReverse(Route.Step step) {
    return step.getVehicleOrientation() == Vehicle.Orientation.BACKWARD;
  }

  public Optional<String> directionForEdge(String startNodeId, String endNodeId) {
    return Optional.ofNullable(directionByEdgeId.get(traversalKey(startNodeId, endNodeId)));
  }

  public Optional<Double> maxSpeedForEdge(String startNodeId, String endNodeId) {
    return Optional.ofNullable(maxSpeedByEdgeId.get(traversalKey(startNodeId, endNodeId)));
  }

  private static String traversalKey(String startNodeId, String endNodeId) {
    return startNodeId + '>' + endNodeId;
  }

  public String compactEdgeId(String edgeId) {
    if (edgeId.length() <= COMPACT_EDGE_ID_LENGTH) {
      return edgeId;
    }

    String compactEdgeId = edgeId.substring(0, EDGE_ID_AFFIX_LENGTH)
        + edgeId.substring(edgeId.length() - EDGE_ID_AFFIX_LENGTH);
    edgeIdByCompactEdgeId.put(compactEdgeId, edgeId);
    return compactEdgeId;
  }

  public String expandEdgeId(String compactEdgeId) {
    return edgeIdByCompactEdgeId.getOrDefault(compactEdgeId, compactEdgeId);
  }

  public String compactActionId(String actionId) {
    if (actionId.length() <= MAX_ACTION_ID_LENGTH) {
      return actionId;
    }

    String compactActionId = shorten(actionId);
    String previous = actionIdByCompactActionId.put(compactActionId, actionId);
    if (previous != null && !previous.equals(actionId)) {
      LOG.warn(
          "Action id '{}' and '{}' both shorten to '{}'; the vehicle cannot tell them apart.",
          previous, actionId, compactActionId
      );
    }
    return compactActionId;
  }

  public String expandActionId(String compactActionId) {
    return actionIdByCompactActionId.getOrDefault(compactActionId, compactActionId);
  }

  public String compactOrderId(String orderId) {
    if (orderId.length() <= MAX_ORDER_ID_LENGTH) {
      return orderId;
    }

    String compactOrderId = allocatedOrderId(orderId);
    String previous = orderIdByCompactOrderId.put(compactOrderId, orderId);
    if (previous != null && !previous.equals(orderId)) {
      LOG.error(
          "Order id '{}' and '{}' both map to '{}'; the vehicle cannot tell them apart.",
          previous, orderId, compactOrderId
      );
    }
    return compactOrderId;
  }

  public String expandOrderId(String compactOrderId) {
    return orderIdByCompactOrderId.getOrDefault(compactOrderId, compactOrderId);
  }

  private String allocatedOrderId(String orderId) {
    String driveOrderIndex = driveOrderIndexSuffix(orderId);
    String orderName = orderId.substring(0, orderId.length() - driveOrderIndex.length());
    return orderTokenFor(orderName) + driveOrderIndex;
  }

  private String orderTokenFor(String orderName) {
    return orderTokenByOrderName.computeIfAbsent(
        orderName,
        name -> Long.toString(orderTokenSequence.getAndIncrement(), Character.MAX_RADIX)
    );
  }

  private static String driveOrderIndexSuffix(String orderId) {
    int separator = orderId.lastIndexOf(DRIVE_ORDER_INDEX_SEPARATOR);
    if (separator < 0 || separator == orderId.length() - 1) {
      return "";
    }

    String suffix = orderId.substring(separator);
    return suffix.chars().skip(1).allMatch(Character::isDigit) ? suffix : "";
  }

  private static String right(String value, int length) {
    return value.length() <= length ? value : value.substring(value.length() - length);
  }

  private static String shorten(String actionId) {
    String shortened = replacePrefix(actionId, CUSTOM_ACTION_ID_PREFIX,
                                     COMPACT_CUSTOM_ACTION_ID_PREFIX);
    if (shortened == null) {
      shortened = replacePrefix(actionId, DESTINATION_ACTION_ID_PREFIX,
                                COMPACT_DESTINATION_ACTION_ID_PREFIX);
    }
    if (shortened == null) {
      shortened = shortenResourceAction(actionId);
    }
    if (shortened == null) {
      shortened = actionId;
    }
    return right(shortened, MAX_ACTION_ID_LENGTH);
  }

  private static String shortenResourceAction(String actionId) {
    int separator = actionId.lastIndexOf(RESOURCE_ACTION_INFIX);
    if (separator < 0) {
      return null;
    }
    String resourceName = actionId.substring(0, separator);
    String actionNumber = actionId.substring(separator + RESOURCE_ACTION_INFIX.length());
    return compactAffixes(resourceName) + "_" + actionNumber;
  }

  private static String compactAffixes(String name) {
    return name.length() <= COMPACT_EDGE_ID_LENGTH
        ? name
        : name.substring(0, EDGE_ID_AFFIX_LENGTH)
            + name.substring(name.length() - EDGE_ID_AFFIX_LENGTH);
  }

  private static String replacePrefix(String actionId, String prefix, String compactPrefix) {
    return actionId.startsWith(prefix)
        ? compactPrefix + actionId.substring(prefix.length())
        : null;
  }
}
