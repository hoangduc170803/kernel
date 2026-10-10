package org.opentcs.fms.kernel.vda5050;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.opentcs.data.model.Path;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Pose;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.DriveOrder;
import org.opentcs.data.order.Route;
import org.opentcs.drivers.vehicle.MovementCommand;

class AubotOrderFilterTest {

  private static final String ORDER_JSON = """
      {
        "orderId": "TO-1-0",
        "nodes": [
          {"nodeId": "0004", "actions": [{"actionId": "0004_action_0"}]},
          {"nodeId": "0005", "actions": [{"actionId": "0005_action_0"}]}
        ],
        "edges": [
          {
            "edgeId": "0004 --- 0005",
            "startNodeId": "0004",
            "endNodeId": "0005",
            "actions": [{"actionId": "Order_destination_action_0"}]
          }
        ]
      }
      """;

  private final ObjectMapper objectMapper = new ObjectMapper();
  private AubotOrderContext context;
  private AubotOrderFilter filter;

  private static Point pointAt(String name, long x, long y) {
    return new Point(name).withPose(new Pose(new Triple(x, y, 0), 0.0));
  }

  @BeforeEach
  void setUp() {
    context = new AubotOrderContext(new AubotSpeeds(40.0, 70.0, 12.0));
    filter = new AubotOrderFilter(context);
  }

  private void recordForwardStep() {
    Point source = pointAt("0004", 0, 0);
    Point destination = pointAt("0005", 1000, 0);
    Path path = new Path("0004 --- 0005", source.getReference(), destination.getReference());
    Route route = new Route(
        List.of(new Route.Step(path, source, destination, Vehicle.Orientation.FORWARD, 0, 1)));
    DriveOrder driveOrder = mock(DriveOrder.class);
    when(driveOrder.getRoute()).thenReturn(route);
    MovementCommand command = mock(MovementCommand.class);
    when(command.getDriveOrder()).thenReturn(driveOrder);

    context.record(command, false);
  }

  private JsonNode filteredOrder() throws Exception {
    return filter.apply(objectMapper.readTree(ORDER_JSON));
  }

  @Test
  void compactsEdgeIdOnTheWire() throws Exception {
    recordForwardStep();

    assertThat(filteredOrder().get("edges").get(0).get("edgeId").asText())
        .isEqualTo("00040005");
  }

  @Test
  void addsDirectionTheStockAdapterNeverSets() throws Exception {
    recordForwardStep();

    assertThat(filteredOrder().get("edges").get(0).get("direction").asText())
        .isEqualTo("X+");
  }

  @Test
  void addsMaxSpeedForTheRecordedLoadState() throws Exception {
    recordForwardStep();

    assertThat(filteredOrder().get("edges").get(0).get("maxSpeed").asDouble())
        .isEqualTo(70.0);
  }

  @Test
  void shortensActionIdsOnBothNodesAndEdges() throws Exception {
    recordForwardStep();
    JsonNode order = filteredOrder();

    assertThat(order.get("nodes").get(0).get("actions").get(0).get("actionId").asText())
        .isEqualTo("0004_0");
    assertThat(order.get("edges").get(0).get("actions").get(0).get("actionId").asText())
        .isEqualTo("ODA_0");
  }

  @Test
  void keepsEveryActionIdWithinTheVehicleLimit() throws Exception {
    recordForwardStep();
    JsonNode order = filteredOrder();

    for (JsonNode holder : List.of(order.get("nodes"), order.get("edges"))) {
      for (JsonNode node : holder) {
        for (JsonNode action : node.get("actions")) {
          assertThat(action.get("actionId").asText().length())
              .isLessThanOrEqualTo(AubotOrderContext.MAX_ACTION_ID_LENGTH);
        }
      }
    }
  }

  @Test
  void leavesEdgeUntouchedWhenNoCommandWasRecorded() throws Exception {
    JsonNode edge = filteredOrder().get("edges").get(0);

    assertThat(edge.get("edgeId").asText()).isEqualTo("00040005");
    assertThat(edge.get("direction")).isNull();
    assertThat(edge.get("maxSpeed")).isNull();
  }

  @Test
  void shortensTheOrderIdTheVehicleRejects() throws Exception {
    String orderJson = """
        {"orderId": "PARK-B300_3_6-0377-5bbb8bf6-7fc2-4df4-a22a-e25f9c20e8b0-0"}
        """;

    String orderId = filter.apply(objectMapper.readTree(orderJson)).get("orderId").asText();

    assertThat(orderId.length()).isLessThanOrEqualTo(AubotOrderContext.MAX_ORDER_ID_LENGTH);
    assertThat(orderId).endsWith("-0");
    assertThat(context.expandOrderId(orderId))
        .isEqualTo("PARK-B300_3_6-0377-5bbb8bf6-7fc2-4df4-a22a-e25f9c20e8b0-0");
  }

  @Test
  void leavesNodePositionsUntouched() throws Exception {
    String orderJson = """
        {"nodes": [{"nodeId": "0009", "nodePosition": {"x": 28.25, "y": -2.3, "mapId": "F1"}}]}
        """;

    JsonNode position = filter.apply(objectMapper.readTree(orderJson))
        .get("nodes").get(0).get("nodePosition");

    assertThat(position.get("x").asDouble()).isEqualTo(28.25);
    assertThat(position.get("y").asDouble()).isEqualTo(-2.3);
  }

  @Test
  void turnsNodePositionsOntoTheVehicleFrameOnAnEastAlignedLayout() throws Exception {
    AubotOrderFilter aligned = new AubotOrderFilter(
        new AubotOrderContext(new AubotSpeeds(40.0, 70.0, 12.0), AubotDirection.EAST));
    String orderJson = """
        {"nodes": [{"nodeId": "0009", "nodePosition": {"x": 28.25, "y": -2.3, "mapId": "F1"}}]}
        """;

    JsonNode position = aligned.apply(objectMapper.readTree(orderJson))
        .get("nodes").get(0).get("nodePosition");

    assertThat(position.get("x").asDouble()).isEqualTo(2.3);
    assertThat(position.get("y").asDouble()).isEqualTo(28.25);
  }

  @Test
  void leavesOrderIdTheVehicleAcceptsUntouched() throws Exception {
    assertThat(filteredOrder().get("orderId").asText()).isEqualTo("TO-1-0");
  }

  @Test
  void compactsTheUuidActionIdsOpenTcsPutsInInstantActions() throws Exception {
    String uuidActionId = "e3a3b69a-cd1f-4788-8a42-e1bd1e51a969";
    String instantActions = """
        {"actions": [{"actionId": "%s", "actionType": "cancelOrder"}]}
        """.formatted(uuidActionId);

    JsonNode result = filter.apply(objectMapper.readTree(instantActions));
    String compacted = result.get("actions").get(0).get("actionId").asText();

    assertThat(compacted.length()).isLessThanOrEqualTo(AubotOrderContext.MAX_ACTION_ID_LENGTH);
    assertThat(context.expandActionId(compacted)).isEqualTo(uuidActionId);
  }
}
