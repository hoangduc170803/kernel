package org.opentcs.fms.kernel.vda5050;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.commadapter.vehicle.vda5050.common.mqtt.IncomingMessage;

class AubotStateRewriterTest {

  private static final String STATE_TOPIC = "aubotagv/2.0.0/AUBOT/V01/state";

  private final ObjectMapper objectMapper = new ObjectMapper();
  private AubotOrderContext context;
  private AubotStateRewriter rewriter;

  @BeforeEach
  void setUp() {
    context = new AubotOrderContext(new AubotSpeeds(40.0, 70.0, 12.0));
    rewriter = new AubotStateRewriter(context);
  }

  private JsonNode rewrittenState(String stateJson) throws Exception {
    IncomingMessage rewritten = rewriter.rewrite(new IncomingMessage(STATE_TOPIC, stateJson));
    return objectMapper.readTree(rewritten.getMessage());
  }

  @Test
  void restoresEdgeIdTheVehicleEchoesBack() throws Exception {
    String compacted = context.compactEdgeId("0004 --- 0005");
    String state = """
        {"edgeStates": [{"edgeId": "%s"}]}
        """.formatted(compacted);

    assertThat(rewrittenState(state).get("edgeStates").get(0).get("edgeId").asText())
        .isEqualTo("0004 --- 0005");
  }

  @Test
  void restoresActionIdTheVehicleEchoesBack() throws Exception {
    String compacted = context.compactActionId("1008_action_0");
    String state = """
        {"actionStates": [{"actionId": "%s", "actionStatus": "FINISHED"}]}
        """.formatted(compacted);

    assertThat(rewrittenState(state).get("actionStates").get(0).get("actionId").asText())
        .isEqualTo("1008_action_0");
  }

  @Test
  void keepsNeighbouringPointActionsApartAcrossTheRoundTrip() throws Exception {
    List<String> originals = List.of("1008_action_0", "2008_action_0", "1007_action_0");
    List<String> compacted = new ArrayList<>();
    for (String original : originals) {
      compacted.add(context.compactActionId(original));
    }

    assertThat(compacted).doesNotHaveDuplicates();

    String state = """
        {"actionStates": [{"actionId": "%s"}, {"actionId": "%s"}, {"actionId": "%s"}]}
        """.formatted(compacted.get(0), compacted.get(1), compacted.get(2));

    JsonNode actionStates = rewrittenState(state).get("actionStates");
    for (int i = 0; i < originals.size(); i++) {
      assertThat(actionStates.get(i).get("actionId").asText()).isEqualTo(originals.get(i));
    }
  }

  @Test
  void restoresOrderIdTheVehicleEchoesBack() throws Exception {
    String transportOrder = "PARK-B300_3_6-0377-5bbb8bf6-7fc2-4df4-a22a-e25f9c20e8b0-0";
    String compacted = context.compactOrderId(transportOrder);
    String state = """
        {"orderId": "%s", "orderUpdateId": 0}
        """.formatted(compacted);

    assertThat(rewrittenState(state).get("orderId").asText()).isEqualTo(transportOrder);
  }

  @Test
  void leavesOrderIdTheKernelNeverSentUntouched() throws Exception {
    String state = """
        {"orderId": "586EREX-0", "orderUpdateId": 21}
        """;

    assertThat(rewrittenState(state).get("orderId").asText()).isEqualTo("586EREX-0");
  }

  @Test
  void leavesIdsTheKernelNeverSentUntouched() throws Exception {
    String state = """
        {"actionStates": [{"actionId": "somethingElse"}], "edgeStates": [{"edgeId": "abcdefgh"}]}
        """;

    JsonNode rewritten = rewrittenState(state);

    assertThat(rewritten.get("actionStates").get(0).get("actionId").asText())
        .isEqualTo("somethingElse");
    assertThat(rewritten.get("edgeStates").get(0).get("edgeId").asText())
        .isEqualTo("abcdefgh");
  }

  @Test
  void dropsTheActionStatesRepeatingAnIdThatAlreadyFinishedOrFailed() throws Exception {
    String state = """
        {"actionStates": [
          {"actionId": "0510_0", "actionStatus": "FINISHED", "actionType": "laserOff"},
          {"actionId": "0511_0", "actionStatus": "FAILED", "actionType": "laserOff"},
          {"actionId": "05110510_0", "actionStatus": "FAILED", "actionType": "laserOff"},
          {"actionId": "05110510_0", "actionStatus": "WAITING", "actionType": "laserOff"},
          {"actionId": "0511_0", "actionStatus": "WAITING", "actionType": "laserOff"},
          {"actionId": "05110510_0", "actionStatus": "WAITING", "actionType": "laserOff"},
          {"actionId": "0510_0", "actionStatus": "WAITING", "actionType": "laserOff"},
          {"actionId": "05110510_0", "actionStatus": "WAITING", "actionType": "laserOff"}
        ]}
        """;

    JsonNode actionStates = rewrittenState(state).get("actionStates");

    List<String> statuses = new ArrayList<>();
    List<String> ids = new ArrayList<>();
    for (JsonNode actionState : actionStates) {
      statuses.add(actionState.get("actionStatus").asText());
      ids.add(actionState.get("actionId").asText());
    }

    assertThat(statuses).containsExactly("FINISHED", "FAILED", "FAILED");
    assertThat(ids).containsExactly("0510_0", "0511_0", "05110510_0");
  }

  @Test
  void keepsAPendingActionThatHasNoTerminalTwin() throws Exception {
    String state = """
        {"actionStates": [
          {"actionId": "05100515_0", "actionStatus": "FINISHED", "actionType": "laserOff"},
          {"actionId": "ODA_0", "actionStatus": "WAITING", "actionType": "liftUp"}
        ]}
        """;

    JsonNode actionStates = rewrittenState(state).get("actionStates");

    assertThat(actionStates.size()).isEqualTo(2);
    assertThat(actionStates.get(1).get("actionStatus").asText()).isEqualTo("WAITING");
  }

  @Test
  void keepsDuplicatesWhileNoneOfThemHasReachedATerminalStatus() throws Exception {
    String state = """
        {"actionStates": [
          {"actionId": "0510_0", "actionStatus": "WAITING", "actionType": "laserOff"},
          {"actionId": "0510_0", "actionStatus": "RUNNING", "actionType": "laserOff"}
        ]}
        """;

    assertThat(rewrittenState(state).get("actionStates").size()).isEqualTo(2);
  }

  @Test
  void exposesAnActiveEmergencyStopAsAFatalVehicleError() throws Exception {
    String state = """
        {"errors": [], "safetyState": {"eStop": "MANUAL", "fieldViolation": false}}
        """;

    JsonNode errors = rewrittenState(state).get("errors");

    assertThat(errors.size()).isEqualTo(1);
    assertThat(errors.get(0).get("errorType").asText()).isEqualTo("eStop");
    assertThat(errors.get(0).get("errorLevel").asText()).isEqualTo("FATAL");
  }

  @Test
  void leavesAReleasedEmergencyStopOutOfVehicleErrors() throws Exception {
    String state = """
        {"errors": [], "safetyState": {"eStop": "NONE", "fieldViolation": false}}
        """;

    assertThat(rewrittenState(state).get("errors").isEmpty()).isTrue();
  }

  @Test
  void doesNotDuplicateAnEmergencyStopAlreadyReportedAsAnError() throws Exception {
    String state = """
        {"errors": [{"errorType": "eStop", "errorLevel": "FATAL"}],
         "safetyState": {"eStop": "MANUAL", "fieldViolation": false}}
        """;

    assertThat(rewrittenState(state).get("errors").size()).isEqualTo(1);
  }

  @Test
  void renamesTheSingularInfoReferenceTheVehicleSends() throws Exception {
    String state = """
        {"information": [{"infoType": "liftSensorStatus", "infoLevel": "INFO",
          "infoReference": [{"referenceKey": "state", "referenceValue": "Down"}]}]}
        """;

    JsonNode entry = rewrittenState(state).get("information").get(0);

    assertThat(entry.get("infoReference")).isNull();
    assertThat(entry.get("infoReferences").get(0).get("referenceValue").asText()).isEqualTo("Down");
  }

  @Test
  void renamesTheSingularErrorReferenceTheVehicleSends() throws Exception {
    String state = """
        {"errors": [{"errorType": "noRouteError", "errorLevel": "WARNING",
          "errorReference": [{"referenceKey": "nodeId", "referenceValue": "0009"}]}]}
        """;

    JsonNode entry = rewrittenState(state).get("errors").get(0);

    assertThat(entry.get("errorReference")).isNull();
    assertThat(entry.get("errorReferences").get(0).get("referenceValue").asText()).isEqualTo("0009");
  }

  @Test
  void stringifiesReferenceValuesTheVehicleSendsAsScalars() throws Exception {
    String state = """
        {"information": [{"infoType": "battery",
          "infoReference": [{"referenceKey": "bmsSOC", "referenceValue": 51},
                            {"referenceKey": "down", "referenceValue": true},
                            {"referenceKey": "current", "referenceValue": -1.25}]}]}
        """;

    JsonNode references = rewrittenState(state).get("information").get(0).get("infoReferences");

    for (JsonNode reference : references) {
      assertThat(reference.get("referenceValue").isTextual()).isTrue();
    }
    assertThat(references.get(0).get("referenceValue").asText()).isEqualTo("51");
    assertThat(references.get(1).get("referenceValue").asText()).isEqualTo("true");
    assertThat(references.get(2).get("referenceValue").asText()).isEqualTo("-1.25");
  }

  @Test
  void leavesSpecCompliantReferenceListsUntouched() {
    IncomingMessage state = new IncomingMessage(
        STATE_TOPIC,
        """
        {"information": [{"infoType": "battery",
          "infoReferences": [{"referenceKey": "bmsSOC", "referenceValue": "51"}]}]}
        """);

    assertThat(rewriter.rewrite(state)).isSameAs(state);
  }

  @Test
  void leavesTheReportedPositionUntouched() throws Exception {
    String state = """
        {"agvPosition": {"positionInitialized": true, "x": 28.25, "y": -2.3, "theta": 0.0}}
        """;

    JsonNode position = rewrittenState(state).get("agvPosition");

    assertThat(position.get("x").asDouble()).isEqualTo(28.25);
    assertThat(position.get("y").asDouble()).isEqualTo(-2.3);
  }

  @Test
  void turnsTheReportedPositionBackOntoTheMapFrameOnAnEastAlignedLayout() throws Exception {
    AubotStateRewriter aligned = new AubotStateRewriter(
        new AubotOrderContext(new AubotSpeeds(40.0, 70.0, 12.0), AubotDirection.EAST));
    String state = """
        {"agvPosition": {"positionInitialized": true, "x": 2.3, "y": 28.25}}
        """;

    JsonNode position = objectMapper
        .readTree(aligned.rewrite(new IncomingMessage(STATE_TOPIC, state)).getMessage())
        .get("agvPosition");

    assertThat(position.get("x").asDouble()).isEqualTo(28.25);
    assertThat(position.get("y").asDouble()).isEqualTo(-2.3);
  }

  @Test
  void returnsTheMapPositionAfterTheVehicleEchoesTheOrderBack() throws Exception {
    AubotOrderContext aligned
        = new AubotOrderContext(new AubotSpeeds(40.0, 70.0, 12.0), AubotDirection.EAST);
    String orderJson = """
        {"nodes": [{"nodeId": "0009", "nodePosition": {"x": 28.25, "y": -2.3, "mapId": "F1"}}]}
        """;

    JsonNode sent = new AubotOrderFilter(aligned).apply(objectMapper.readTree(orderJson))
        .get("nodes").get(0).get("nodePosition");
    String echoed = """
        {"agvPosition": {"positionInitialized": true, "x": %s, "y": %s}}
        """.formatted(sent.get("x").asDouble(), sent.get("y").asDouble());

    JsonNode position = objectMapper
        .readTree(
            new AubotStateRewriter(aligned)
                .rewrite(new IncomingMessage(STATE_TOPIC, echoed)).getMessage())
        .get("agvPosition");

    assertThat(position.get("x").asDouble()).isEqualTo(28.25);
    assertThat(position.get("y").asDouble()).isEqualTo(-2.3);
  }

  @Test
  void readsTheHeadingOfAVehicleFacingNorthOnAnEastAlignedLayout() throws Exception {
    AubotStateRewriter aligned = new AubotStateRewriter(
        new AubotOrderContext(new AubotSpeeds(40.0, 70.0, 12.0), AubotDirection.EAST));
    String state = """
        {"agvPosition": {"positionInitialized": true, "x": 28.3, "y": -2.32, "theta": -1.5522}}
        """;

    double theta = objectMapper
        .readTree(aligned.rewrite(new IncomingMessage(STATE_TOPIC, state)).getMessage())
        .get("agvPosition").get("theta").asDouble();

    assertThat(Math.toDegrees(theta)).isCloseTo(88.9, within(0.1));
  }

  @Test
  void turnsTheCompassHeadingTheVehicleReportsIntoAnOpenTcsAngle() throws Exception {
    String state = """
        {"agvPosition": {"positionInitialized": true, "x": 28.25, "y": -2.28, "theta": 0.0056}}
        """;

    double theta = rewrittenState(state).get("agvPosition").get("theta").asDouble();

    assertThat(Math.toDegrees(theta)).isCloseTo(89.7, within(0.1));
  }

  @Test
  void readsAHeadingOfMinusNinetyDegreesAsPointingWest() throws Exception {
    String state = """
        {"agvPosition": {"positionInitialized": true, "x": 28.25, "y": -2.28, "theta": -1.5485}}
        """;

    double theta = rewrittenState(state).get("agvPosition").get("theta").asDouble();

    assertThat(Math.toDegrees(theta)).isCloseTo(178.7, within(0.1));
  }

  @Test
  void leavesMessagesOnOtherTopicsUntouched() {
    IncomingMessage connection = new IncomingMessage(
        "aubotagv/2.0.0/AUBOT/V01/connection",
        "{\"connectionState\": \"ONLINE\"}");

    assertThat(rewriter.rewrite(connection)).isSameAs(connection);
  }

  @Test
  void leavesUnparseableStateUntouched() {
    IncomingMessage broken = new IncomingMessage(STATE_TOPIC, "{not json");

    assertThat(rewriter.rewrite(broken)).isSameAs(broken);
  }
}
