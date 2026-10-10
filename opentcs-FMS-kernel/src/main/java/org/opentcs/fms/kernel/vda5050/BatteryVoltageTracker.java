package org.opentcs.fms.kernel.vda5050;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.Locale;
import java.util.Optional;
import javax.annotation.Nullable;
import org.opentcs.commadapter.vehicle.vda5050.common.mqtt.IncomingMessage;

public class BatteryVoltageTracker {

  /**
   * Vehicle property holding the last reported battery voltage, in volts with one decimal.
   */
  public static final String PROPKEY_BATTERY_VOLTAGE = "fms:batteryVoltage";

  private final ObjectMapper objectMapper = new ObjectMapper();
  @Nullable
  private String lastReported;

  public BatteryVoltageTracker() {
  }

  public Optional<String> changedVoltage(IncomingMessage message) {
    if (!message.getTopic().endsWith("/state")) {
      return Optional.empty();
    }
    Optional<String> voltage = voltageOf(message.getMessage()).map(BatteryVoltageTracker::format);
    if (voltage.isEmpty() || voltage.get().equals(lastReported)) {
      return Optional.empty();
    }
    lastReported = voltage.get();
    return voltage;
  }

  private Optional<Double> voltageOf(String stateJson) {
    try {
      JsonNode voltage = objectMapper.readTree(stateJson)
          .path("batteryState")
          .path("batteryVoltage");
      return voltage.isNumber() ? Optional.of(voltage.asDouble()) : Optional.empty();
    }
    catch (JsonProcessingException exc) {
      return Optional.empty();
    }
  }

  private static String format(double volts) {
    return String.format(Locale.ROOT, "%.1f", volts);
  }
}
