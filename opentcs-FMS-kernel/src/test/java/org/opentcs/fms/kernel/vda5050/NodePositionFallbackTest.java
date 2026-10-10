package org.opentcs.fms.kernel.vda5050;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;
import org.opentcs.commadapter.vehicle.vda5050.common.mqtt.IncomingMessage;

class NodePositionFallbackTest {

  private static final String STATE_TOPIC = "aubotagv/2.0.0/AUBOT/V01/state";

  private final NodePositionFallback fallback = new NodePositionFallback();

  private static IncomingMessage state(String json) {
    return new IncomingMessage(STATE_TOPIC, json);
  }

  @Test
  void usesTheLastNodeWhenTheStateCarriesNoPosition() {
    assertThat(fallback.nodeWithoutPosition(state("{\"lastNodeId\": \"0378\"}")))
        .contains("0378");
    assertThat(
        fallback.nodeWithoutPosition(state("{\"lastNodeId\": \"0378\", \"agvPosition\": null}"))
    ).contains("0378");
  }

  @Test
  void leavesAReportedPositionAlone() {
    assertThat(
        fallback.nodeWithoutPosition(
            state("{\"lastNodeId\": \"0378\", \"agvPosition\": {\"x\": 1.0, \"y\": 2.0}}")
        )
    ).isEmpty();
  }

  @Test
  void needsALastNode() {
    assertThat(fallback.nodeWithoutPosition(state("{\"lastNodeId\": \"\"}"))).isEmpty();
    assertThat(fallback.nodeWithoutPosition(state("{}"))).isEmpty();
    assertThat(fallback.nodeWithoutPosition(state("not json"))).isEmpty();
  }

  @Test
  void ignoresMessagesOtherThanState() {
    assertThat(
        fallback.nodeWithoutPosition(
            new IncomingMessage("aubotagv/2.0.0/AUBOT/V01/visualization", "{\"lastNodeId\": \"1\"}")
        )
    ).isEmpty();
  }
}
