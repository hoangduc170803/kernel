package org.opentcs.fms.kernel.vda5050;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.commadapter.vehicle.vda5050.common.mqtt.IncomingMessage;

class ErrorDescriptionTrackerTest {

  private static final String STATE_TOPIC = "aubotagv/2.0.0/AUBOT/V01/state";

  private ErrorDescriptionTracker tracker;

  @BeforeEach
  void setUp() {
    tracker = new ErrorDescriptionTracker();
  }

  private static IncomingMessage state(String json) {
    return new IncomingMessage(STATE_TOPIC, json);
  }

  @Test
  void reportsTheVehiclesOwnDescriptionPerErrorType() {
    assertThat(tracker.changedDescriptions(state("""
        {"errors": [
          {"errorType": "eStop", "errorLevel": "FATAL",
           "errorDescription": "Emergency stop button is pressed"},
          {"errorType": "adapterLostNavigation", "errorLevel": "FATAL",
           "errorDescription": "Lost navigation at 2001"}
        ]}
        """))).contains(
            "{\"adapterLostNavigation\":\"Lost navigation at 2001\","
                + "\"eStop\":\"Emergency stop button is pressed\"}");
  }

  @Test
  void leavesOutErrorsTheVehicleDidNotDescribe() {
    assertThat(tracker.changedDescriptions(state("""
        {"errors": [
          {"errorType": "orderError", "errorLevel": "WARNING"},
          {"errorType": "eStop", "errorLevel": "FATAL", "errorDescription": "  "}
        ]}
        """))).contains("{}");
  }

  @Test
  void staysQuietWhileTheDescriptionsAreUnchanged() {
    String withError = "{\"errors\": [{\"errorType\": \"eStop\", \"errorDescription\": \"pressed\"}]}";
    tracker.changedDescriptions(state(withError));

    assertThat(tracker.changedDescriptions(state(withError))).isEmpty();
  }

  @Test
  void reportsAnEmptyObjectOnceTheErrorsAreGone() {
    tracker.changedDescriptions(
        state("{\"errors\": [{\"errorType\": \"eStop\", \"errorDescription\": \"pressed\"}]}"));

    assertThat(tracker.changedDescriptions(state("{\"errors\": []}"))).contains("{}");
    assertThat(tracker.changedDescriptions(state("{}"))).isEmpty();
  }

  @Test
  void ignoresUnreadableStatesAndOtherTopics() {
    assertThat(tracker.changedDescriptions(state("not json"))).isEmpty();
    assertThat(tracker.changedDescriptions(new IncomingMessage(
        "aubotagv/2.0.0/AUBOT/V01/visualization",
        "{\"errors\": [{\"errorType\": \"eStop\", \"errorDescription\": \"pressed\"}]}")))
        .isEmpty();
  }
}
