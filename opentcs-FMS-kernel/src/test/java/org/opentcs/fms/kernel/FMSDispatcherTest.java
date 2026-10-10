package org.opentcs.fms.kernel;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import org.junit.jupiter.api.Test;

class FMSDispatcherTest {

  private static final List<String> REVISITING_ROUTE
      = List.of("0007", "0008", "0009", "0008", "0009", "3073", "3049");

  @Test
  void suffixAnchorsAtTheVisitTheVehicleIsHeadingTo() {
    assertThat(FMSDispatcher.suffixFrom(REVISITING_ROUTE, "0009", 4))
        .containsExactly("0009", "3073", "3049");
  }

  @Test
  void suffixIgnoresAnAlreadyTravelledVisitOfTheSamePoint() {
    assertThat(FMSDispatcher.suffixFrom(REVISITING_ROUTE, "0009", 4))
        .isNotEqualTo(REVISITING_ROUTE.subList(2, REVISITING_ROUTE.size()));
  }

  @Test
  void suffixWithoutProgressAnchorsAtTheFirstVisit() {
    assertThat(FMSDispatcher.suffixFrom(REVISITING_ROUTE, "0009", 0))
        .containsExactly("0009", "0008", "0009", "3073", "3049");
  }

  @Test
  void suffixIsEmptyWhenThePointLiesOnlyBehindTheVehicle() {
    assertThat(FMSDispatcher.suffixFrom(REVISITING_ROUTE, "0007", 4)).isEmpty();
  }
}
