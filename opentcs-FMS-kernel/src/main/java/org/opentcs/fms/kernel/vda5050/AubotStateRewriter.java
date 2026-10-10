package org.opentcs.fms.kernel.vda5050;

import static java.util.Objects.requireNonNull;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Predicate;
import javax.annotation.Nullable;
import org.opentcs.commadapter.vehicle.vda5050.common.mqtt.IncomingMessage;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class AubotStateRewriter {

  private static final Logger LOG = LoggerFactory.getLogger(AubotStateRewriter.class);

  private static final List<ReferenceList> REFERENCE_LISTS = List.of(
      new ReferenceList("information", "infoReference", "infoReferences"),
      new ReferenceList("errors", "errorReference", "errorReferences"));
  private static final List<String> REFERENCE_FIELDS = List.of("referenceKey", "referenceValue");
  private static final Set<String> VISUALIZATION_KEYS = Set.of(
      "headerId", "timestamp", "version", "manufacturer",
      "serialNumber", "agvPosition", "velocity");
  private static final Set<String> TERMINAL_ACTION_STATUSES = Set.of("FINISHED", "FAILED");

  private final ObjectMapper objectMapper = new ObjectMapper();
  private final AubotOrderContext context;
  @Nullable
  private String droppedActionIdsLastLogged;

  public AubotStateRewriter(AubotOrderContext context) {
    this.context = requireNonNull(context, "context");
  }

  public IncomingMessage rewrite(IncomingMessage message) {
    requireNonNull(message, "message");

    if (message.getTopic().endsWith("/state")) {
      return rewriteParsed(message, this::normaliseState);
    }
    if (message.getTopic().endsWith("/visualization")) {
      return rewriteParsed(message, this::normaliseVisualization);
    }
    return message;
  }

  private IncomingMessage rewriteParsed(IncomingMessage message, Predicate<JsonNode> normaliser) {
    JsonNode payload;
    try {
      payload = objectMapper.readTree(message.getMessage());
    } catch (com.fasterxml.jackson.core.JsonProcessingException exc) {
      return message;
    }

    if (!normaliser.test(payload)) {
      return message;
    }

    return new IncomingMessage(message.getTopic(), payload.toString());
  }

  private boolean normaliseState(JsonNode state) {
    return dedupeActionStates(state)
        | expandOrderId(state)
        | expandEdgeIds(state)
        | expandActionIds(state)
        | normaliseEmergencyStop(state)
        | normaliseReferenceLists(state)
        | dropUninitializedPosition(state)
        | rotatePositionOntoMapFrame(state)
        | rotateThetaOntoMapFrame(state);
  }

  private boolean normaliseEmergencyStop(JsonNode state) {
    if (!(state instanceof ObjectNode stateNode)) {
      return false;
    }

    String eStop = stateNode.path("safetyState").path("eStop").asText("");
    if (eStop.isBlank() || eStop.equalsIgnoreCase("NONE")) {
      return false;
    }

    ArrayNode errors;
    if (stateNode.get("errors") instanceof ArrayNode existingErrors) {
      errors = existingErrors;
    } else {
      errors = stateNode.putArray("errors");
    }
    for (JsonNode error : errors) {
      if (error.path("errorType").asText().equalsIgnoreCase("eStop")) {
        return false;
      }
    }

    ObjectNode error = errors.addObject();
    error.put("errorType", "eStop");
    error.put("errorLevel", "FATAL");
    error.put("errorDescription", "Safety state eStop: " + eStop);
    return true;
  }

  private boolean normaliseVisualization(JsonNode visualization) {
    return retainVisualizationKeys(visualization)
        | dropUninitializedPosition(visualization)
        | rotatePositionOntoMapFrame(visualization)
        | rotateThetaOntoMapFrame(visualization);
  }

  private boolean rotatePositionOntoMapFrame(JsonNode payload) {
    if (!(payload instanceof ObjectNode payloadNode)) {
      return false;
    }

    if (!(payloadNode.get("agvPosition") instanceof ObjectNode positionNode)) {
      return false;
    }
    JsonNode x = positionNode.get("x");
    JsonNode y = positionNode.get("y");
    if (x == null || !x.isNumber() || y == null || !y.isNumber()) {
      return false;
    }
    AubotDirection.Coordinates mapFrame = context.toMapCoordinates(x.asDouble(), y.asDouble());
    if (mapFrame.x() == x.asDouble() && mapFrame.y() == y.asDouble()) {
      return false;
    }
    positionNode.put("x", mapFrame.x());
    positionNode.put("y", mapFrame.y());
    return true;
  }

  private boolean rotateThetaOntoMapFrame(JsonNode payload) {
    if (!(payload instanceof ObjectNode payloadNode)) {
      return false;
    }

    if (!(payloadNode.get("agvPosition") instanceof ObjectNode positionNode)) {
      return false;
    }
    JsonNode theta = positionNode.get("theta");
    if (theta == null || !theta.isNumber()) {
      return false;
    }
    double mapAngle = context.mapAngleOf(theta.asDouble());
    if (mapAngle == theta.asDouble()) {
      return false;
    }
    positionNode.put("theta", mapAngle);
    return true;
  }

  private boolean dropUninitializedPosition(JsonNode payload) {
    if (!(payload instanceof ObjectNode payloadNode)) {
      return false;
    }

    JsonNode position = payloadNode.get("agvPosition");
    if (position == null || position.path("positionInitialized").asBoolean(true)) {
      return false;
    }

    payloadNode.remove("agvPosition");
    return true;
  }

  private boolean expandOrderId(JsonNode state) {
    if (!(state instanceof ObjectNode stateNode)) {
      return false;
    }

    JsonNode orderId = stateNode.get("orderId");
    if (orderId == null || !orderId.isTextual()) {
      return false;
    }
    String expanded = context.expandOrderId(orderId.asText());
    if (expanded.equals(orderId.asText())) {
      return false;
    }
    stateNode.put("orderId", expanded);
    return true;
  }

  private boolean retainVisualizationKeys(JsonNode visualization) {
    if (!(visualization instanceof ObjectNode visualizationNode)) {
      return false;
    }

    List<String> extraneousKeys = new ArrayList<>();
    visualizationNode.fieldNames().forEachRemaining(fieldName -> {
      if (!VISUALIZATION_KEYS.contains(fieldName)) {
        extraneousKeys.add(fieldName);
      }
    });
    visualizationNode.remove(extraneousKeys);

    return !extraneousKeys.isEmpty();
  }

  private boolean dedupeActionStates(JsonNode state) {
    if (!(state.get("actionStates") instanceof ArrayNode actionStates)) {
      return false;
    }

    Set<String> terminated = new HashSet<>();
    for (JsonNode actionState : actionStates) {
      String actionId = actionIdOf(actionState);
      if (actionId != null && isTerminal(actionState)) {
        terminated.add(actionId);
      }
    }

    List<String> dropped = new ArrayList<>();
    for (int index = actionStates.size() - 1; index >= 0; index--) {
      JsonNode actionState = actionStates.get(index);
      String actionId = actionIdOf(actionState);
      if (actionId != null && !isTerminal(actionState) && terminated.contains(actionId)) {
        actionStates.remove(index);
        dropped.add(actionId);
      }
    }

    logDroppedActionStates(dropped);
    return !dropped.isEmpty();
  }

  private void logDroppedActionStates(List<String> dropped) {
    if (dropped.isEmpty()) {
      droppedActionIdsLastLogged = null;
      return;
    }

    List<String> ids = new ArrayList<>(dropped);
    ids.sort(null);
    String signature = String.join(",", ids);
    if (signature.equals(droppedActionIdsLastLogged)) {
      return;
    }
    droppedActionIdsLastLogged = signature;
    LOG.info(
        "[FMS actionstate] dropping {} action state(s) the vehicle repeats under an id that"
            + " already finished or failed {}: duplicates are what keep it reporting EXECUTING"
            + " with no order left to execute",
        ids.size(), ids);
  }

  private static boolean isTerminal(JsonNode actionState) {
    return TERMINAL_ACTION_STATUSES.contains(actionState.path("actionStatus").asText());
  }

  @Nullable
  private static String actionIdOf(JsonNode actionState) {
    JsonNode actionId = actionState.get("actionId");
    return actionId != null && actionId.isTextual() ? actionId.asText() : null;
  }

  private boolean expandActionIds(JsonNode state) {
    JsonNode actionStates = state.get("actionStates");
    if (actionStates == null || !actionStates.isArray()) {
      return false;
    }

    boolean rewritten = false;
    for (JsonNode actionState : actionStates) {
      if (!(actionState instanceof ObjectNode actionStateNode)) {
        continue;
      }
      JsonNode actionId = actionStateNode.get("actionId");
      if (actionId == null || !actionId.isTextual()) {
        continue;
      }
      String expanded = context.expandActionId(actionId.asText());
      if (!expanded.equals(actionId.asText())) {
        actionStateNode.put("actionId", expanded);
        rewritten = true;
      }
    }
    return rewritten;
  }

  private boolean expandEdgeIds(JsonNode state) {
    JsonNode edgeStates = state.get("edgeStates");
    if (edgeStates == null || !edgeStates.isArray()) {
      return false;
    }

    boolean rewritten = false;
    for (JsonNode edgeState : edgeStates) {
      if (!(edgeState instanceof ObjectNode edgeStateNode)) {
        continue;
      }
      JsonNode edgeId = edgeStateNode.get("edgeId");
      if (edgeId == null || !edgeId.isTextual()) {
        continue;
      }
      String expanded = context.expandEdgeId(edgeId.asText());
      if (!expanded.equals(edgeId.asText())) {
        edgeStateNode.put("edgeId", expanded);
        rewritten = true;
      }
    }
    return rewritten;
  }

  private boolean normaliseReferenceLists(JsonNode state) {
    boolean rewritten = false;
    for (ReferenceList referenceList : REFERENCE_LISTS) {
      JsonNode entries = state.get(referenceList.field());
      if (entries == null || !entries.isArray()) {
        continue;
      }
      for (JsonNode entry : entries) {
        if (entry instanceof ObjectNode entryNode) {
          rewritten |= normaliseEntry(entryNode, referenceList);
        }
      }
    }
    return rewritten;
  }

  private boolean normaliseEntry(ObjectNode entry, ReferenceList referenceList) {
    boolean rewritten = false;

    JsonNode singular = entry.remove(referenceList.singular());
    if (singular != null) {
      entry.set(referenceList.plural(), singular);
      rewritten = true;
    }

    JsonNode references = entry.get(referenceList.plural());
    if (references == null || !references.isArray()) {
      return rewritten;
    }
    for (JsonNode reference : references) {
      if (reference instanceof ObjectNode referenceNode) {
        rewritten |= stringifyReferenceFields(referenceNode);
      }
    }
    return rewritten;
  }

  private boolean stringifyReferenceFields(ObjectNode reference) {
    boolean rewritten = false;
    for (String field : REFERENCE_FIELDS) {
      JsonNode value = reference.get(field);
      if (value == null || value.isNull() || value.isTextual()) {
        continue;
      }
      reference.put(field, value.isContainerNode() ? value.toString() : value.asText());
      rewritten = true;
    }
    return rewritten;
  }

  private record ReferenceList(String field, String singular, String plural) {
  }
}
