package org.opentcs.fms.kernel.vda5050;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.util.Optional;
import java.util.TreeMap;
import javax.annotation.Nullable;
import org.opentcs.commadapter.vehicle.vda5050.common.mqtt.IncomingMessage;

public class ErrorDescriptionTracker {

  /**
   * Vehicle property holding what the vehicle itself says about each active error: a JSON object
   * mapping errorType to the errorDescription of its state message, {@code {}} when it reports
   * none. The error types alone are in the adapter's own vda5050:errors.* properties.
   */
  public static final String PROPKEY_ERROR_DESCRIPTIONS = "fms:errorDescriptions";

  private final ObjectMapper objectMapper = new ObjectMapper();
  @Nullable
  private String lastReported;

  public ErrorDescriptionTracker() {
  }

  public Optional<String> changedDescriptions(IncomingMessage message) {
    if (!message.getTopic().endsWith("/state")) {
      return Optional.empty();
    }
    Optional<String> descriptions = descriptionsOf(message.getMessage());
    if (descriptions.isEmpty() || descriptions.get().equals(lastReported)) {
      return Optional.empty();
    }
    lastReported = descriptions.get();
    return descriptions;
  }

  private Optional<String> descriptionsOf(String stateJson) {
    JsonNode errors;
    try {
      errors = objectMapper.readTree(stateJson).path("errors");
    }
    catch (JsonProcessingException exc) {
      return Optional.empty();
    }

    TreeMap<String, String> byErrorType = new TreeMap<>();
    for (JsonNode error : errors) {
      String errorType = error.path("errorType").asText("").trim();
      String description = error.path("errorDescription").asText("").trim();
      if (!errorType.isEmpty() && !description.isEmpty()) {
        byErrorType.putIfAbsent(errorType, description);
      }
    }

    ObjectNode encoded = objectMapper.createObjectNode();
    byErrorType.forEach(encoded::put);
    return Optional.of(encoded.toString());
  }
}
