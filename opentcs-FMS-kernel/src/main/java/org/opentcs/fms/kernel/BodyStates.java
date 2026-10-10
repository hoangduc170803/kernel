package org.opentcs.fms.kernel;

import java.util.Collection;
import java.util.HashMap;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function;

import org.opentcs.data.model.Point;
import org.opentcs.data.model.Pose;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.DriveOrder;
import org.opentcs.data.order.Route;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.drivers.vehicle.LoadHandlingDevice;
import org.opentcs.fms.kernel.mapf.BodyState;
import org.opentcs.fms.routing.Roadmap;

public final class BodyStates {

  private static String lastBodySummary;

  private static final org.slf4j.Logger logger = org.slf4j.LoggerFactory.getLogger(BodyStates.class);

  private BodyStates() {
  }

  public static BodyState of(
      Collection<Vehicle> vehicles, Map<String, String> positions,
      Function<Vehicle, Optional<TransportOrder>> orderOf) {
    Map<String, Integer> axisByVehicle = new HashMap<>();
    Set<String> loadedVehicles = new HashSet<>();
    int fromPose = 0;
    int fromRoute = 0;
    int unknown = 0;
    int routeAgreed = 0;
    int turnedInPlace = 0;
    List<String> unexplained = new ArrayList<>();

    for (Vehicle vehicle : vehicles) {
      String name = vehicle.getName();
      String position = positions.get(name);
      if (position == null) {
        continue;
      }
      Optional<TransportOrder> order = orderOf.apply(vehicle);
      if (carriesCargo(vehicle)) {
        loadedVehicles.add(name);
      }

      int byPose = standsOn(vehicle, position)
          ? axisOfPose(vehicle.getPose())
          : Roadmap.DIR_NONE;
      int arriving = noseArrivingAt(order, position);
      int leaving = noseLeavingFrom(order, position);
      if (byPose != Roadmap.DIR_NONE) {
        if (byPose == arriving) {
          routeAgreed++;
        } else if (byPose == leaving) {
          turnedInPlace++;
        } else if (arriving != Roadmap.DIR_NONE || leaving != Roadmap.DIR_NONE) {
          unexplained.add(String.format(
              "%s@%s pose=%.0f°/%s arriving=%s leaving=%s",
              name, position, vehicle.getPose().getOrientationAngle(), axisName(byPose),
              axisName(arriving), axisName(leaving)));
        }
      }

      int axis = byPose != Roadmap.DIR_NONE ? byPose : arriving;
      if (axis == Roadmap.DIR_NONE) {
        unknown++;
        continue;
      }
      axisByVehicle.put(name, axis);
      if (byPose != Roadmap.DIR_NONE) {
        fromPose++;
      } else {
        fromRoute++;
      }
    }

    String summary = fromPose + "|" + fromRoute + "|" + unknown + "|" + loadedVehicles.size()
        + "|" + sorted(loadedVehicles) + "|" + routeAgreed + "|" + turnedInPlace;
    if (!summary.equals(lastBodySummary)) {
      lastBodySummary = summary;
      logger.info(
        "[MAPF body] {} noses measured from the pose of a vehicle standing on the very cell the"
            + " plan starts from, {} derived from the route step that lands on that cell,"
            + " {} unknown (a vehicle with no nose can never be seen to pivot) | {} carrying a"
            + " load {} | pose matches the nose it arrived on for {}, already matches the nose"
            + " it is about to leave on for {} (turned in place, still a rectangle)",
        fromPose, fromRoute, unknown, loadedVehicles.size(), sorted(loadedVehicles),
        routeAgreed, turnedInPlace);
    }
    if (!unexplained.isEmpty()) {
      logger.error(
          "[MAPF body] {} vehicle(s) standing on the cell their plan starts from whose reported"
              + " orientation matches neither the nose they arrived on nor the one they are about"
              + " to leave on: {}. A mid-pivot vehicle explains a mismatch with the arrival nose"
              + " alone; matching neither means the pose frame and the roadmap frame are rotated"
              + " relative to each other, which makes every nose wrong rather than merely unknown.",
          unexplained.size(), unexplained);
    }
    return new BodyState(
        Map.copyOf(axisByVehicle), Set.copyOf(loadedVehicles));
  }

  private static String axisName(int axis) {
    return axis == Roadmap.DIR_NONE ? "none" : String.valueOf(axis);
  }

  static int axisOfPose(Pose pose) {
    if (pose == null || Double.isNaN(pose.getOrientationAngle())) {
      return Roadmap.DIR_NONE;
    }
    double degrees = ((pose.getOrientationAngle() % 360) + 360) % 360;
    if (degrees < 45 || degrees >= 315) {
      return Roadmap.DIR_POS_X;
    }
    if (degrees < 135) {
      return Roadmap.DIR_POS_Y;
    }
    if (degrees < 225) {
      return Roadmap.DIR_NEG_X;
    }
    return Roadmap.DIR_NEG_Y;
  }

  private static boolean standsOn(Vehicle vehicle, String position) {
    return vehicle.getCurrentPosition() != null
        && position.equals(vehicle.getCurrentPosition().getName());
  }

  static int noseArrivingAt(Optional<TransportOrder> order, String frontier) {
    return noseOfStepWhere(order, frontier, false);
  }

  static int noseLeavingFrom(Optional<TransportOrder> order, String frontier) {
    return noseOfStepWhere(order, frontier, true);
  }

  private static int noseOfStepWhere(
      Optional<TransportOrder> order, String frontier, boolean matchSource) {
    DriveOrder current = order.map(TransportOrder::getCurrentDriveOrder).orElse(null);
    if (current == null || current.getRoute() == null) {
      return Roadmap.DIR_NONE;
    }
    for (Route.Step step : current.getRoute().getSteps()) {
      Point source = step.getSourcePoint();
      Point destination = step.getDestinationPoint();
      if (source == null || destination == null) {
        continue;
      }
      if (!(matchSource ? source : destination).getName().equals(frontier)) {
        continue;
      }
      return noseDriving(step, source, destination);
    }
    return Roadmap.DIR_NONE;
  }

  private static int noseDriving(Route.Step step, Point source, Point destination) {
    int travel = axisBetween(source, destination);
    if (travel == Roadmap.DIR_NONE
        || step.getVehicleOrientation() != Vehicle.Orientation.BACKWARD) {
      return travel;
    }
    return Roadmap.opposite(travel);
  }

  private static int axisBetween(Point source, Point destination) {
    if (source.getPose() == null || destination.getPose() == null
        || source.getPose().getPosition() == null
        || destination.getPose().getPosition() == null) {
      return Roadmap.DIR_NONE;
    }
    long dx = destination.getPose().getPosition().getX() - source.getPose().getPosition().getX();
    long dy = destination.getPose().getPosition().getY() - source.getPose().getPosition().getY();
    if (Math.abs(dx) == Math.abs(dy)) {
      return Roadmap.DIR_NONE;
    }
    if (Math.abs(dx) > Math.abs(dy)) {
      return dx > 0 ? Roadmap.DIR_POS_X : Roadmap.DIR_NEG_X;
    }
    return dy > 0 ? Roadmap.DIR_POS_Y : Roadmap.DIR_NEG_Y;
  }

  private static java.util.List<String> sorted(java.util.Set<String> names) {
    java.util.List<String> out = new java.util.ArrayList<>(names);
    java.util.Collections.sort(out);
    return out;
  }

  private static boolean carriesCargo(Vehicle vehicle) {
    return vehicle.getLoadHandlingDevices().stream().anyMatch(LoadHandlingDevice::isFull);
  }
}
