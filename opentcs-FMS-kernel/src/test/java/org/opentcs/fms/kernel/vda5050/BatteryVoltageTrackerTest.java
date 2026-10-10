package org.opentcs.fms.kernel.vda5050;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.commadapter.vehicle.vda5050.common.mqtt.IncomingMessage;

class BatteryVoltageTrackerTest {

  private static final String STATE_TOPIC = "aubotagv/2.0.0/AUBOT/V01/state";

  private BatteryVoltageTracker tracker;

  @BeforeEach
  void setUp() {
    tracker = new BatteryVoltageTracker();
  }

  private static IncomingMessage state(String json) {
    return new IncomingMessage(STATE_TOPIC, json);
  }

  @Test
  void reportsTheVoltageRoundedToOneDecimal() {
    assertThat(tracker.changedVoltage(state("""
        {"batteryState": {"batteryCharge": 80, "batteryVoltage": 48.26}}
        """))).contains("48.3");
  }

  @Test
  void staysQuietWhileTheRoundedVoltageIsUnchanged() {
    tracker.changedVoltage(state("{\"batteryState\": {\"batteryVoltage\": 48.26}}"));

    assertThat(tracker.changedVoltage(state("{\"batteryState\": {\"batteryVoltage\": 48.31}}")))
        .isEmpty();
    assertThat(tracker.changedVoltage(state("{\"batteryState\": {\"batteryVoltage\": 48.12}}")))
        .contains("48.1");
  }

  @Test
  void ignoresStatesWithoutAVoltage() {
    assertThat(tracker.changedVoltage(state("{\"batteryState\": {\"batteryCharge\": 80}}")))
        .isEmpty();
    assertThat(tracker.changedVoltage(state("{}"))).isEmpty();
    assertThat(tracker.changedVoltage(state("not json"))).isEmpty();
  }

  @Test
  void ignoresMessagesOtherThanState() {
    assertThat(tracker.changedVoltage(new IncomingMessage(
        "aubotagv/2.0.0/AUBOT/V01/visualization",
        "{\"batteryState\": {\"batteryVoltage\": 48.2}}"))).isEmpty();
  }
}
