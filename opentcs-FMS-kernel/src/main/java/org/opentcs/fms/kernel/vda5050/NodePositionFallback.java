package org.opentcs.fms.kernel.vda5050;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.Optional;
import org.opentcs.commadapter.vehicle.vda5050.common.mqtt.IncomingMessage;

public class NodePositionFallback {

  private final ObjectMapper objectMapper = new ObjectMapper();

  public NodePositionFallback() {
  }

  public Optional<String> nodeWithoutPosition(IncomingMessage message) {
    if (!message.getTopic().endsWith("/state")) {
      return Optional.empty();
    }
    JsonNode state;
    try {
      state = objectMapper.readTree(message.getMessage());
    }
    catch (JsonProcessingException exc) {
      return Optional.empty();
    }
    if (state.path("agvPosition").isObject()) {
      return Optional.empty();
    }
    String lastNodeId = state.path("lastNodeId").asText("").trim();
    return lastNodeId.isEmpty() ? Optional.empty() : Optional.of(lastNodeId);
  }
}
