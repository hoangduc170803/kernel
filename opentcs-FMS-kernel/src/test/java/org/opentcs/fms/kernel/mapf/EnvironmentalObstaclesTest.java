package org.opentcs.fms.kernel.mapf;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.opentcs.data.model.Couple;
import org.opentcs.data.model.Envelope;
import org.opentcs.data.model.EnvironmentalEntity;
import org.opentcs.data.model.Pose;
import org.opentcs.data.model.Triple;
import org.opentcs.fms.routing.pibt.Footprint;

class EnvironmentalObstaclesTest {

  private static final Envelope CARGO_800_BY_740 = new Envelope(List.of(
      new Couple(-400, -370),
      new Couple(400, -370),
      new Couple(400, 370),
      new Couple(-400, 370),
      new Couple(-400, -370)));

  private static EnvironmentalEntity cargoAt(long x, long y, double angle) {
    return new EnvironmentalEntity(
        "cargo", CARGO_800_BY_740, new Pose(new Triple(x, y, 0), angle));
  }

  @Test
  void theEnvelopeIsPlacedAtThePose() {
    Footprint.Obstacle obstacle = EnvironmentalObstacles.obstacleOf(cargoAt(5000, -2000, 0));

    assertThat(obstacle.minX()).isCloseTo(4600, within(1e-6));
    assertThat(obstacle.maxX()).isCloseTo(5400, within(1e-6));
    assertThat(obstacle.minY()).isCloseTo(-2370, within(1e-6));
    assertThat(obstacle.maxY()).isCloseTo(-1630, within(1e-6));
  }

  @Test
  void aQuarterTurnSwapsTheExtents() {
    Footprint.Obstacle obstacle = EnvironmentalObstacles.obstacleOf(cargoAt(0, 0, 90));

    assertThat(obstacle.maxX() - obstacle.minX()).isCloseTo(740, within(1e-6));
    assertThat(obstacle.maxY() - obstacle.minY()).isCloseTo(800, within(1e-6));
  }

  @Test
  void onlyLiveObjectsToBeRespectedBlockVehicles() {
    EnvironmentalEntity cargo = cargoAt(0, 0, 0);

    assertThat(EnvironmentalObstacles.blocksVehicles(cargo)).isTrue();
    assertThat(EnvironmentalObstacles.blocksVehicles(cargo.withRetired(true))).isFalse();
    assertThat(EnvironmentalObstacles.blocksVehicles(
        cargo.withIntegrationLevel(EnvironmentalEntity.IntegrationLevel.TO_BE_NOTICED)))
        .isFalse();
    assertThat(EnvironmentalObstacles.blocksVehicles(
        cargo.withType(EnvironmentalEntity.Type.ZONE)))
        .isFalse();
  }
}
