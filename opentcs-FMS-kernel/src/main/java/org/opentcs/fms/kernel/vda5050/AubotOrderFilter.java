package org.opentcs.fms.kernel.vda5050;

import static java.util.Objects.requireNonNull;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.util.function.Function;

public class AubotOrderFilter
    implements
    Function<JsonNode, JsonNode> {

  private final AubotOrderContext context;

  public AubotOrderFilter(AubotOrderContext context) {
    this.context = requireNonNull(context, "context");
  }

  @Override
  public JsonNode apply(JsonNode treeModel) {
    requireNonNull(treeModel, "treeModel");

    compactOrderId(treeModel);
    rewriteEdges(treeModel.get("edges"));
    rewriteNodePositions(treeModel.get("nodes"));
    compactActionIds(treeModel.get("edges"));
    compactActionIds(treeModel.get("nodes"));
    compactActionIdsIn(treeModel.get("actions"));

    return treeModel;
  }

  private void compactOrderId(JsonNode treeModel) {
    if (!(treeModel instanceof ObjectNode orderNode)) {
      return;
    }

    JsonNode orderId = orderNode.get("orderId");
    if (orderId == null || !orderId.isTextual()) {
      return;
    }
    orderNode.put("orderId", context.compactOrderId(orderId.asText()));
  }

  private void rewriteEdges(JsonNode edges) {
    if (edges == null || !edges.isArray()) {
      return;
    }

    for (JsonNode edge : edges) {
      if (!(edge instanceof ObjectNode edgeNode)) {
        continue;
      }
      JsonNode edgeId = edgeNode.get("edgeId");
      if (edgeId == null || !edgeId.isTextual()) {
        continue;
      }
      String startNodeId = edgeNode.path("startNodeId").asText(null);
      String endNodeId = edgeNode.path("endNodeId").asText(null);
      if (startNodeId != null && endNodeId != null) {
        context.directionForEdge(startNodeId, endNodeId)
            .ifPresent(direction -> edgeNode.put("direction", direction));
        context.maxSpeedForEdge(startNodeId, endNodeId)
            .ifPresent(maxSpeed -> edgeNode.put("maxSpeed", maxSpeed));
      }
      edgeNode.put("edgeId", context.compactEdgeId(edgeId.asText()));
    }
  }

  private void rewriteNodePositions(JsonNode nodes) {
    if (nodes == null || !nodes.isArray()) {
      return;
    }

    for (JsonNode node : nodes) {
      if (!(node.get("nodePosition") instanceof ObjectNode positionNode)) {
        continue;
      }
      JsonNode x = positionNode.get("x");
      JsonNode y = positionNode.get("y");
      if (x == null || !x.isNumber() || y == null || !y.isNumber()) {
        continue;
      }
      AubotDirection.Coordinates vehicleFrame
          = context.toVehicleCoordinates(x.asDouble(), y.asDouble());
      positionNode.put("x", vehicleFrame.x());
      positionNode.put("y", vehicleFrame.y());
    }
  }

  private void compactActionIds(JsonNode actionHolders) {
    if (actionHolders == null || !actionHolders.isArray()) {
      return;
    }

    for (JsonNode holder : actionHolders) {
      compactActionIdsIn(holder.get("actions"));
    }
  }

  private void compactActionIdsIn(JsonNode actions) {
    if (actions == null || !actions.isArray()) {
      return;
    }

    for (JsonNode action : actions) {
      if (!(action instanceof ObjectNode actionNode)) {
        continue;
      }
      JsonNode actionId = actionNode.get("actionId");
      if (actionId == null || !actionId.isTextual()) {
        continue;
      }
      actionNode.put("actionId", context.compactActionId(actionId.asText()));
    }
  }
}
