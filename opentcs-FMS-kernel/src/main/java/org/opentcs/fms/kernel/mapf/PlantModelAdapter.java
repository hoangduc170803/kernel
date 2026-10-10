package org.opentcs.fms.kernel.mapf;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.opentcs.data.model.Location;
import org.opentcs.data.model.Path;
import org.opentcs.data.model.PlantModel;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Pose;
import org.opentcs.data.model.Triple;
import org.opentcs.fms.kernel.VehicleOperations;
import org.opentcs.data.model.visualization.VisualLayout;
import org.opentcs.fms.routing.LaneAxis;
import org.opentcs.fms.routing.Roadmap;

public final class PlantModelAdapter {

  private PlantModelAdapter() {
  }

  public static Roadmap.Model toModel(PlantModel plantModel) {
    List<Point> sortedPoints = new ArrayList<>(plantModel.getPoints());
    sortedPoints.sort(Comparator.comparing(Point::getName));
    List<Path> sortedPaths = new ArrayList<>(plantModel.getPaths());
    sortedPaths.sort(Comparator.comparing(Path::getName));

    List<Roadmap.Point> points = new ArrayList<>(sortedPoints.size());
    for (Point p : sortedPoints) {
      double x = 0;
      double y = 0;
      double z = 0;
      Pose pose = p.getPose();
      if (pose != null && pose.getPosition() != null) {
        Triple pos = pose.getPosition();
        x = pos.getX();
        y = pos.getY();
        z = pos.getZ();
      }
      points.add(new Roadmap.Point(p.getName(), x, y, z));
    }

    List<Roadmap.Path> edges = new ArrayList<>();
    for (Path path : sortedPaths) {
      if (path.isLocked()) {
        continue;
      }
      String src = path.getSourcePoint().getName();
      String dst = path.getDestinationPoint().getName();
      long len = path.getLength();
      if (path.getMaxVelocity() > 0) {
        edges.add(new Roadmap.Path(path.getName(), src, dst, len, false, false));
      }
      if (path.getMaxReverseVelocity() > 0) {
        edges.add(new Roadmap.Path(path.getName() + "~rev", dst, src, len, false, true));
      }
    }
    return new Roadmap.Model(
        points, edges, cargoSlotPoints(plantModel), dropOffPoints(plantModel),
        laneDirectionDeclared(plantModel));
  }

  private static boolean laneDirectionDeclared(PlantModel plantModel) {
    VisualLayout layout = plantModel.getVisualLayout();
    return layout != null
        && layout.getProperty(LaneAxis.LAYOUT_DIRECTION_PROPERTY) != null;
  }

  private static Set<String> dropOffPoints(PlantModel plantModel) {
    Set<String> slots = new HashSet<>();
    for (Location location : plantModel.getLocations()) {
      boolean dropOff = plantModel.getLocationType(location.getType().getName())
          .map(type -> VehicleOperations.isDropOff(type.getAllowedOperations()))
          .orElse(false);
      if (!dropOff) {
        continue;
      }
      for (Location.Link link : location.getAttachedLinks()) {
        slots.add(link.getPoint().getName());
      }
    }
    return slots;
  }

  private static Set<String> cargoSlotPoints(PlantModel plantModel) {
    Set<String> slots = new HashSet<>();
    for (Location location : plantModel.getLocations()) {
      boolean waitingCargo = plantModel.getLocationType(location.getType().getName())
          .map(type -> VehicleOperations.isPickUp(type.getAllowedOperations()))
          .orElse(false);
      if (!waitingCargo) {
        continue;
      }
      for (Location.Link link : location.getAttachedLinks()) {
        slots.add(link.getPoint().getName());
      }
    }
    return slots;
  }

  public static Roadmap toRoadmap(PlantModel plantModel) {
    return new Roadmap(toModel(plantModel));
  }
}
