package org.opentcs.fms.kernel.vda5050;

import static java.util.Objects.requireNonNull;
import static org.opentcs.commadapter.vehicle.vda5050.v2_0.ObjectProperties.PROPKEY_VEHICLE_INTERFACE_NAME;
import static org.opentcs.commadapter.vehicle.vda5050.v2_0.ObjectProperties.PROPKEY_VEHICLE_MANUFACTURER;
import static org.opentcs.commadapter.vehicle.vda5050.v2_0.ObjectProperties.PROPKEY_VEHICLE_MAX_STEPS_BASE;
import static org.opentcs.commadapter.vehicle.vda5050.v2_0.ObjectProperties.PROPKEY_VEHICLE_SERIAL_NUMBER;
import static org.opentcs.commadapter.vehicle.vda5050.v2_0.ObjectProperties.PROPKEY_VEHICLE_VERSION;

import jakarta.inject.Inject;
import jakarta.inject.Provider;
import java.util.Arrays;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.ScheduledExecutorService;
import org.opentcs.commadapter.vehicle.vda5050.CommAdapterConfiguration;
import org.opentcs.commadapter.vehicle.vda5050.common.JsonBinder;
import org.opentcs.commadapter.vehicle.vda5050.common.mqtt.MqttClientManager;
import org.opentcs.commadapter.vehicle.vda5050.v2_0.CommAdapterComponentsFactory;
import org.opentcs.commadapter.vehicle.vda5050.v2_0.IncomingMessageFilter;
import org.opentcs.commadapter.vehicle.vda5050.v2_0.MessageValidator;
import org.opentcs.commadapter.vehicle.vda5050.v2_0.ordermapping.UnsupportedPropertiesExtractor;
import org.opentcs.components.kernel.services.TCSObjectService;
import org.opentcs.customizations.kernel.KernelExecutor;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.model.visualization.VisualLayout;
import org.opentcs.drivers.vehicle.VehicleCommAdapter;
import org.opentcs.drivers.vehicle.VehicleCommAdapterDescription;
import org.opentcs.drivers.vehicle.VehicleCommAdapterFactory;

public class AubotCommAdapterFactory
    implements
    VehicleCommAdapterFactory {

  private static final org.slf4j.Logger logger = org.slf4j.LoggerFactory.getLogger(AubotCommAdapterFactory.class);

  static final String PROPKEY_LAYOUT_DIRECTION = "direction";
  static final String MAX_STEPS_BASE = "1";

  private final ScheduledExecutorService kernelExecutor;
  private final TCSObjectService objectService;
  private final CommAdapterComponentsFactory componentsFactory;
  private final MqttClientManager clientManager;
  private final Provider<MessageValidator> messageValidatorProvider;
  private final Provider<IncomingMessageFilter> incomingMessageFilterProvider;
  private final Provider<JsonBinder> jsonBinderProvider;
  private final CommAdapterConfiguration configuration;
  private final UnsupportedPropertiesExtractor unsupportedPropertiesExtractor;

  private boolean initialized;

  @Inject
  public AubotCommAdapterFactory(
      @KernelExecutor ScheduledExecutorService kernelExecutor,
      TCSObjectService objectService,
      CommAdapterComponentsFactory componentsFactory,
      MqttClientManager clientManager,
      Provider<MessageValidator> messageValidatorProvider,
      Provider<IncomingMessageFilter> incomingMessageFilterProvider,
      Provider<JsonBinder> jsonBinderProvider,
      CommAdapterConfiguration configuration,
      UnsupportedPropertiesExtractor unsupportedPropertiesExtractor) {
    this.kernelExecutor = requireNonNull(kernelExecutor, "kernelExecutor");
    this.objectService = requireNonNull(objectService, "objectService");
    this.componentsFactory = requireNonNull(componentsFactory, "componentsFactory");
    this.clientManager = requireNonNull(clientManager, "clientManager");
    this.messageValidatorProvider = requireNonNull(messageValidatorProvider, "messageValidatorProvider");
    this.incomingMessageFilterProvider = requireNonNull(incomingMessageFilterProvider, "incomingMessageFilterProvider");
    this.jsonBinderProvider = requireNonNull(jsonBinderProvider, "jsonBinderProvider");
    this.configuration = requireNonNull(configuration, "configuration");
    this.unsupportedPropertiesExtractor = requireNonNull(unsupportedPropertiesExtractor,
        "unsupportedPropertiesExtractor");
  }

  @Override
  public void initialize() {
    initialized = true;
  }

  @Override
  public boolean isInitialized() {
    return initialized;
  }

  @Override
  public void terminate() {
    initialized = false;
  }

  @Override
  public VehicleCommAdapterDescription getDescription() {
    return new AubotCommAdapterDescription();
  }

  @Override
  public boolean providesAdapterFor(Vehicle vehicle) {
    requireNonNull(vehicle, "vehicle");

    String version = vehicle.getProperty(PROPKEY_VEHICLE_VERSION);

    return version != null
        && version.trim().contains("2.0")
        && vehicle.getProperty(PROPKEY_VEHICLE_INTERFACE_NAME) != null
        && vehicle.getProperty(PROPKEY_VEHICLE_MANUFACTURER) != null
        && vehicle.getProperty(PROPKEY_VEHICLE_SERIAL_NUMBER) != null;
  }

  @Override
  public VehicleCommAdapter getAdapterFor(Vehicle vehicle) {
    requireNonNull(vehicle, "vehicle");

    if (!providesAdapterFor(vehicle)) {
      return null;
    }

    return new AubotCommAdapter(
        vehicle.withProperty(PROPKEY_VEHICLE_MAX_STEPS_BASE, MAX_STEPS_BASE),
        kernelExecutor,
        componentsFactory,
        clientManager,
        messageValidatorProvider.get(),
        incomingMessageFilterProvider.get(),
        jsonBinderProvider.get(),
        configuration,
        unsupportedPropertiesExtractor,
        layoutDirection(),
        this::pointPosition);
  }

  private Optional<Triple> pointPosition(String pointName) {
    return objectService.fetch(Point.class, pointName)
        .map(point -> point.getPose().getPosition());
  }

  private AubotDirection layoutDirection() {
    String configured = objectService.fetch(VisualLayout.class).stream()
        .map(layout -> layout.getProperty(PROPKEY_LAYOUT_DIRECTION))
        .filter(Objects::nonNull)
        .findFirst()
        .orElse(null);

    if (configured == null) {
      logger.warn(
          "[Aubot layout] no '{}' property on any visual layout, so every reported heading is"
              + " taken to be in the map frame already. If the QR codes are laid at a right angle"
              + " to the map this is wrong for every vehicle at once, and an empty vehicle read"
              + " broadside needs 212.5mm where it really needs 420mm.",
          PROPKEY_LAYOUT_DIRECTION);
      return null;
    }

    AubotDirection direction = AubotDirection.fromCode(configured);
    if (direction == null) {
      logger.error(
          "[Aubot layout] '{}' is \"{}\", which is none of {}. Headings are left unrotated, which"
              + " looks identical to a correct setup until a footprint decision is made on a"
              + " heading that is a quarter turn out.",
          PROPKEY_LAYOUT_DIRECTION, configured,
          Arrays.stream(AubotDirection.values()).map(AubotDirection::getCode).toList());
      return null;
    }

    logger.info(
        "[Aubot layout] headings are rotated onto the map frame by '{}'=\"{}\".",
        PROPKEY_LAYOUT_DIRECTION, direction.getCode());
    return direction;
  }

  private static class AubotCommAdapterDescription
      extends
      VehicleCommAdapterDescription {

    AubotCommAdapterDescription() {
    }

    @Override
    public String getDescription() {
      return "AUBOT VDA5050 2.0 (topic " + AubotCommAdapter.AGV_TOPIC_VERSION + ")";
    }

    @Override
    public boolean isSimVehicleCommAdapter() {
      return false;
    }
  }
}
