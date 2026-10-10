package org.opentcs.fms.kernel.transformers;

import org.opentcs.data.model.Path;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.Route;
import org.opentcs.drivers.vehicle.MovementCommand;
import org.opentcs.drivers.vehicle.MovementCommandTransformer;
import org.opentcs.fms.kernel.vda5050.AubotDirection;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class LaserActionTransformer
    implements
    MovementCommandTransformer {
  private static final Logger LOG = LoggerFactory.getLogger(LaserActionTransformer.class);
  static final String LASER_ACTION_PROPERTY = "vda5050:action.laser";
  static final String LASER_ACTION_BLOCKING_TYPE_PROPERTY = "vda5050:action.laser.blockingType";
  static final String LASER_ON_ACTION = "laserOn";
  static final String LASER_OFF_ACTION = "laserOff";
  static final String POINT_LASER_PROPERTY = "laser";
  static final String POINT_LASER_OFF_VALUE = "off";
  static final String LIFT_UP_OPERATION = "liftUp";
  static final String LIFT_DOWN_OPERATION = "liftDown";
  static final String LEAVE_GOODS_OPERATION = "leaveGoods";
  static final String LEG_PROPERTY = "wes:leg";
  static final String APPROACH_LEG = "APPROACH";
  static final int HANDLING_END_SEGMENT_STEPS = 3;

  private static final String BLOCKING_TYPE_NONE = "NONE";

  public LaserActionTransformer() {
  }

  @Override
  public MovementCommand apply(MovementCommand command) {
    Route.Step step = command.getStep();
    boolean laserOffForStep = isInActionHandlingEndSegment(command, step)
        || turnsAtDestination(command, step);
    String actionAtDestination = laserActionFor(step.getDestinationPoint(), laserOffForStep);

    return command.withStep(
        step
            .withSourcePoint(sourcePointWithLaserAction(step, laserOffForStep))
            .withDestinationPoint(
                pointWithLaserAction(step.getDestinationPoint(), actionAtDestination))
            .withPath(pathWithLaserAction(step.getPath(), actionAtDestination)));
  }

  private static String laserActionFor(Point point, boolean laserOffForStep) {
    return laserOffForStep || requiresLaserOff(point)
        ? LASER_OFF_ACTION
        : LASER_ON_ACTION;
  }

  private static boolean requiresLaserOff(Point point) {
    return point != null
        && (point.getType() == Point.Type.PARK_POSITION
            || POINT_LASER_OFF_VALUE.equalsIgnoreCase(point.getProperty(POINT_LASER_PROPERTY)));
  }

  private static boolean turnsAtDestination(MovementCommand command, Route.Step step) {
    Route.Step nextStep = stepAfter(command, step);
    if (nextStep == null) {
      return false;
    }

    AubotDirection incoming = noseDirectionOf(step);
    AubotDirection outgoing = noseDirectionOf(nextStep);
    if (incoming == null || outgoing == null || incoming == outgoing) {
      return false;
    }

    return incoming.reverse() != outgoing;
  }

  private static AubotDirection noseDirectionOf(Route.Step step) {
    if (step.getSourcePoint() == null || step.getDestinationPoint() == null) {
      return null;
    }

    return AubotDirection.between(
        step.getSourcePoint(),
        step.getDestinationPoint(),
        step.getVehicleOrientation() == Vehicle.Orientation.BACKWARD);
  }

  private static Route.Step stepAfter(MovementCommand command, Route.Step step) {
    Route route = command.getDriveOrder().getRoute();
    if (route == null) {
      return null;
    }

    for (Route.Step candidate : route.getSteps()) {
      if (candidate.getRouteIndex() == step.getRouteIndex() + 1) {
        return candidate;
      }
    }
    return null;
  }

  private static boolean isInActionHandlingEndSegment(
      MovementCommand command,
      Route.Step step) {
    return (isActionHandlingFinalOperation(command) || isApproachLeg(command))
        && remainingRouteStepsInclusive(command, step) <= HANDLING_END_SEGMENT_STEPS;
  }

  private static boolean isApproachLeg(MovementCommand command) {
    return APPROACH_LEG.equals(command.getProperties().get(LEG_PROPERTY));
  }

  private static boolean isActionHandlingFinalOperation(MovementCommand command) {
    String finalOperation = command.getFinalOperation();
    return LIFT_UP_OPERATION.equals(finalOperation)
        || LIFT_DOWN_OPERATION.equals(finalOperation)
        || LEAVE_GOODS_OPERATION.equals(finalOperation);
  }

  private static int remainingRouteStepsInclusive(
      MovementCommand command,
      Route.Step step) {
    Route route = command.getDriveOrder().getRoute();
    if (route == null) {
      return Integer.MAX_VALUE;
    }

    return Math.max(0, route.getSteps().size() - step.getRouteIndex());
  }

  private static Point sourcePointWithLaserAction(Route.Step step, boolean laserOffForStep) {
    Point sourcePoint = step.getSourcePoint();
    if (sourcePoint == null || step.getRouteIndex() != 0) {
      return sourcePoint;
    }

    return pointWithLaserAction(sourcePoint, laserActionFor(sourcePoint, laserOffForStep));
  }

  private static Point pointWithLaserAction(Point point, String laserAction) {
    return point
        .withProperty(LASER_ACTION_PROPERTY, laserAction)
        .withProperty(LASER_ACTION_BLOCKING_TYPE_PROPERTY, BLOCKING_TYPE_NONE);
  }

  private static Path pathWithLaserAction(Path path, String laserAction) {
    if (path == null) {
      return null;
    }

    return path
        .withProperty(LASER_ACTION_PROPERTY, laserAction)
        .withProperty(LASER_ACTION_BLOCKING_TYPE_PROPERTY, BLOCKING_TYPE_NONE);
  }
}
