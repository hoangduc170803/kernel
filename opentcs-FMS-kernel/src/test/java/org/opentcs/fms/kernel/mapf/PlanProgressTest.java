package org.opentcs.fms.kernel.mapf;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;

class PlanProgressTest {

  @Test
  void aClaimThatIsTheRestOfTheRoutePointsAtItsFirstCell() {
    Optional<PlanProgress> progress = PlanProgress.of(
        List.of("0134", "0124", "0114", "0104", "0086"), List.of("0104", "0086"));

    assertThat(progress).map(PlanProgress::step).contains(3);
    assertThat(progress).map(PlanProgress::pointUnderRequest).contains("0104");
  }

  @Test
  void futureDriveOrdersAfterTheRouteDoNotMoveTheStep() {
    Optional<PlanProgress> progress = PlanProgress.of(
        List.of("0510", "0515"), List.of("0515", "0510", "0531"));

    assertThat(progress).map(PlanProgress::pointUnderRequest).contains("0515");
  }

  @Test
  void aLegThePlanNeverCoveredHasNoProgress() {
    assertThat(PlanProgress.of(List.of("0510", "0515"), List.of("0510", "0531"))).isEmpty();
  }

  @Test
  void nothingLeftToAllocateMeansTheLastStep() {
    assertThat(PlanProgress.of(List.of("0510", "0515"), List.of()))
        .map(PlanProgress::pointUnderRequest).contains("0515");
  }

  @Test
  void aCellLeftBehindIsPassedButACellVisitedAgainLaterIsNot() {
    PlanProgress backIntoTheLane =
        PlanProgress.of(List.of("0510", "0515"), List.of("0515", "0510")).orElseThrow();
    PlanProgress revisiting = PlanProgress.of(
        List.of("0089", "0090", "0091", "0090", "0108"), List.of("0091", "0090", "0108"))
        .orElseThrow();

    assertThat(backIntoTheLane.hasAlreadyPassedAnyOf(List.of("0510"))).isTrue();
    assertThat(revisiting.hasAlreadyPassedAnyOf(List.of("0090"))).isFalse();
  }
}
