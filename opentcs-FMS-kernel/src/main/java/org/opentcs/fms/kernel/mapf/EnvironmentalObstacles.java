package org.opentcs.fms.kernel.mapf;

import java.util.Comparator;
import java.util.List;
import org.opentcs.components.kernel.services.TCSObjectService;
import org.opentcs.data.model.Couple;
import org.opentcs.data.model.EnvironmentalEntity;
import org.opentcs.data.model.Pose;
import org.opentcs.fms.routing.pibt.Footprint;

final class EnvironmentalObstacles {

  private EnvironmentalObstacles() {
  }

  static List<Footprint.Obstacle> respectedIn(TCSObjectService objects) {
    return objects.fetch(EnvironmentalEntity.class, EnvironmentalObstacles::blocksVehicles)
        .stream()
        .sorted(Comparator.comparing(EnvironmentalEntity::getName))
        .map(EnvironmentalObstacles::obstacleOf)
        .toList();
  }

  static boolean blocksVehicles(EnvironmentalEntity entity) {
    return !entity.isRetired()
        && entity.getType() == EnvironmentalEntity.Type.OBJECT
        && entity.getIntegrationLevel() == EnvironmentalEntity.IntegrationLevel.TO_BE_RESPECTED
        && !entity.getEnvelope().getVertices().isEmpty();
  }

  static Footprint.Obstacle obstacleOf(EnvironmentalEntity entity) {
    Pose pose = entity.getPose();
    double radians = Math.toRadians(pose.getOrientationAngle());
    double cos = Math.cos(radians);
    double sin = Math.sin(radians);
    List<Couple> vertices = entity.getEnvelope().getVertices();
    double[] xs = new double[vertices.size()];
    double[] ys = new double[vertices.size()];
    for (int i = 0; i < vertices.size(); i++) {
      Couple vertex = vertices.get(i);
      xs[i] = pose.getPosition().getX() + vertex.getX() * cos - vertex.getY() * sin;
      ys[i] = pose.getPosition().getY() + vertex.getX() * sin + vertex.getY() * cos;
    }
    return Footprint.Obstacle.bounding(entity.getName(), xs, ys);
  }
}
