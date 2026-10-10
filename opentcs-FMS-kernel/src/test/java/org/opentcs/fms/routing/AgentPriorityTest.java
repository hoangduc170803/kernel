package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;

class AgentPriorityTest {

  @Test
  void previousOrderBreaksAnOtherwiseExactTie() {
    AgentPriority previouslyFirst = new AgentPriority(0, 0, 8, 0, 0.0023f, 4);
    AgentPriority previouslySecond = new AgentPriority(0, 0, 8, 0, 0.0023f, 5);

    assertThat(previouslyFirst).isGreaterThan(previouslySecond);
  }

  @Test
  void currentDistanceTieStillOutranksPreviousOrder() {
    AgentPriority currentlyFarther = new AgentPriority(0, 0, 8, 0, 0.0024f, 5);
    AgentPriority previouslyFirst = new AgentPriority(0, 0, 8, 0, 0.0023f, 4);

    assertThat(currentlyFarther).isGreaterThan(previouslyFirst);
  }
}
