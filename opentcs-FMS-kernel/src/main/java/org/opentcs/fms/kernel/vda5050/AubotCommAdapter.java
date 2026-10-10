package org.opentcs.fms.kernel.vda5050;

import static org.opentcs.commadapter.vehicle.vda5050.v2_0.ObjectProperties.PROPKEY_VEHICLE_INTERFACE_NAME;
import static org.opentcs.commadapter.vehicle.vda5050.v2_0.ObjectProperties.PROPKEY_VEHICLE_MANUFACTURER;
import static org.opentcs.commadapter.vehicle.vda5050.v2_0.ObjectProperties.PROPKEY_VEHICLE_SERIAL_NUMBER;
import static org.opentcs.commadapter.vehicle.vda5050.v2_0.ObjectProperties.PROPKEY_VEHICLE_TOPIC_PREFIX;

import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.ScheduledExecutorService;
import java.util.function.Function;
import javax.annotation.Nullable;
import org.opentcs.commadapter.vehicle.vda5050.CommAdapterConfiguration;
import org.opentcs.commadapter.vehicle.vda5050.common.JsonBinder;
import org.opentcs.commadapter.vehicle.vda5050.common.mqtt.IncomingMessage;
import org.opentcs.commadapter.vehicle.vda5050.common.mqtt.MqttClientManager;
import org.opentcs.commadapter.vehicle.vda5050.v2_0.CommAdapterComponentsFactory;
import org.opentcs.commadapter.vehicle.vda5050.v2_0.CommAdapterImpl;
import org.opentcs.commadapter.vehicle.vda5050.v2_0.IncomingMessageFilter;
import org.opentcs.commadapter.vehicle.vda5050.v2_0.MessageValidator;
import org.opentcs.commadapter.vehicle.vda5050.v2_0.MqttSetting;
import org.opentcs.commadapter.vehicle.vda5050.v2_0.ordermapping.UnsupportedPropertiesExtractor;
import org.opentcs.data.model.Pose;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.LoadHandlingDevice;
import org.opentcs.drivers.vehicle.MovementCommand;

public class AubotCommAdapter
    extends
    CommAdapterImpl {

  static final String AGV_TOPIC_VERSION = "2.0.0";
  static final String PROPKEY_SPEED_LOADED = "aubot:speed.loaded";
  static final String PROPKEY_SPEED_UNLOADED = "aubot:speed.unloaded";
  static final String PROPKEY_SPEED_TURN = "aubot:speed.turn";
  static final double DEFAULT_SPEED_LOADED = 40.0;
  static final double DEFAULT_SPEED_UNLOADED = 50.0;
  static final double DEFAULT_SPEED_TURN = 40.0;

  private final AubotOrderContext orderContext;
  private final AubotStateRewriter stateRewriter;
  private final BatteryVoltageTracker voltageTracker = new BatteryVoltageTracker();
  private final ErrorDescriptionTracker errorDescriptionTracker = new ErrorDescriptionTracker();
  private final NodePositionFallback nodePositionFallback = new NodePositionFallback();
  private final Function<String, Optional<Triple>> nodePositions;

  @SuppressWarnings("this-escape")
  public AubotCommAdapter(
      Vehicle vehicle,
      ScheduledExecutorService kernelExecutor,
      CommAdapterComponentsFactory componentsFactory,
      MqttClientManager clientManager,
      MessageValidator messageValidator,
      IncomingMessageFilter incomingMessageFilter,
      JsonBinder jsonBinder,
      CommAdapterConfiguration configuration,
      UnsupportedPropertiesExtractor unsupportedPropertiesExtractor,
      @Nullable AubotDirection layoutDirection,
      Function<String, Optional<Triple>> nodePositions) {
    super(
        vehicle,
        aubotMqttSetting(vehicle),
        messageValidator,
        kernelExecutor,
        componentsFactory,
        clientManager,
        incomingMessageFilter,
        jsonBinder,
        configuration,
        unsupportedPropertiesExtractor);

    this.orderContext = new AubotOrderContext(
        new AubotSpeeds(
            speedProperty(vehicle, PROPKEY_SPEED_LOADED, DEFAULT_SPEED_LOADED),
            speedProperty(vehicle, PROPKEY_SPEED_UNLOADED, DEFAULT_SPEED_UNLOADED),
            speedProperty(vehicle, PROPKEY_SPEED_TURN, DEFAULT_SPEED_TURN)),
        layoutDirection);
    this.stateRewriter = new AubotStateRewriter(orderContext);
    this.nodePositions = nodePositions;

    jsonBinder.setFilter(
        componentsFactory
            .createUnsupportedPropertiesFilter(vehicle, unsupportedPropertiesExtractor)
            .andThen(new AubotOrderFilter(orderContext)));
  }

  @Override
  public synchronized void sendCommand(MovementCommand cmd)
      throws IllegalArgumentException {
    orderContext.record(cmd, isLoaded());
    super.sendCommand(cmd);
  }

  private boolean isLoaded() {
    return getProcessModel().getLoadHandlingDevices().stream()
        .anyMatch(LoadHandlingDevice::isFull);
  }

  static double speedProperty(Vehicle vehicle, String key, double fallback) {
    String value = vehicle.getProperty(key);
    if (value == null) {
      return fallback;
    }
    try {
      return Double.parseDouble(value.trim());
    } catch (NumberFormatException exc) {
      return fallback;
    }
  }

  @Override
  public synchronized void onIncomingMessage(IncomingMessage message) {
    IncomingMessage rewritten = stateRewriter.rewrite(message);
    voltageTracker.changedVoltage(rewritten).ifPresent(this::reportBatteryVoltage);
    errorDescriptionTracker.changedDescriptions(rewritten)
        .ifPresent(this::reportErrorDescriptions);
    super.onIncomingMessage(rewritten);
    nodePositionFallback.nodeWithoutPosition(rewritten)
        .ifPresent(node -> getExecutor().execute(() -> placeOnNode(node)));
  }

  private void placeOnNode(String node) {
    nodePositions.apply(node).ifPresent(nodePosition -> {
      Pose current = getProcessModel().getPose();
      if (current != null && Objects.equals(current.getPosition(), nodePosition)) {
        return;
      }
      double orientation = current == null ? Double.NaN : current.getOrientationAngle();
      getProcessModel().setPose(new Pose(nodePosition, orientation));
    });
  }

  private void reportBatteryVoltage(String volts) {
    getExecutor().execute(
        () -> getProcessModel().setProperty(BatteryVoltageTracker.PROPKEY_BATTERY_VOLTAGE, volts));
  }

  private void reportErrorDescriptions(String descriptions) {
    getExecutor().execute(
        () -> getProcessModel().setProperty(
            ErrorDescriptionTracker.PROPKEY_ERROR_DESCRIPTIONS, descriptions));
  }

  static MqttSetting aubotMqttSetting(Vehicle vehicle) {
    return MqttSetting
        .forVehicle(vehicle.withProperty(PROPKEY_VEHICLE_TOPIC_PREFIX, agvTopicPrefix(vehicle)))
        .orElseThrow(IllegalStateException::new);
  }

  static String agvTopicPrefix(Vehicle vehicle) {
    return vehicle.getProperty(PROPKEY_VEHICLE_INTERFACE_NAME)
        + "/" + AGV_TOPIC_VERSION
        + "/" + vehicle.getProperty(PROPKEY_VEHICLE_MANUFACTURER)
        + "/" + vehicle.getProperty(PROPKEY_VEHICLE_SERIAL_NUMBER);
  }
}
