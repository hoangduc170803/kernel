package org.opentcs.fms.kernel;

import static io.javalin.apibuilder.ApiBuilder.path;
import static io.javalin.apibuilder.ApiBuilder.post;
import static io.javalin.apibuilder.ApiBuilder.put;
import static java.util.Objects.requireNonNull;

import io.javalin.apibuilder.EndpointGroup;
import io.javalin.http.Context;

import jakarta.inject.Inject;

import java.util.Set;
import java.util.concurrent.ScheduledExecutorService;

import org.opentcs.access.LocalKernel;
import org.opentcs.components.kernel.services.InternalTCSObjectService;
import org.opentcs.customizations.kernel.KernelExecutor;
import org.opentcs.data.ObjectUnknownException;
import org.opentcs.data.model.Vehicle;
import org.opentcs.kernel.extensions.servicewebapi.common.HttpConstants;
import org.opentcs.kernel.extensions.servicewebapi.common.JsonBinder;
import org.opentcs.kernel.extensions.servicewebapi.common.KernelExecutorWrapper;
import org.opentcs.kernel.extensions.servicewebapi.v1.EnvironmentalEntityHandler;
import org.opentcs.kernel.extensions.servicewebapi.v1.LocationHandler;
import org.opentcs.kernel.extensions.servicewebapi.v1.PathHandler;
import org.opentcs.kernel.extensions.servicewebapi.v1.PeripheralHandler;
import org.opentcs.kernel.extensions.servicewebapi.v1.PeripheralJobDispatcherHandler;
import org.opentcs.kernel.extensions.servicewebapi.v1.PeripheralJobHandler;
import org.opentcs.kernel.extensions.servicewebapi.v1.PlantModelHandler;
import org.opentcs.kernel.extensions.servicewebapi.v1.StatusEventDispatcher;
import org.opentcs.kernel.extensions.servicewebapi.v1.TransportOrderDispatcherHandler;
import org.opentcs.kernel.extensions.servicewebapi.v1.TransportOrderHandler;
import org.opentcs.kernel.extensions.servicewebapi.v1.V1RequestHandler;
import org.opentcs.kernel.extensions.servicewebapi.v1.VehicleHandler;
import org.opentcs.kernel.extensions.servicewebapi.v1.converter.OrderSequenceConverter;
import org.opentcs.kernel.extensions.servicewebapi.v1.converter.PeripheralAttachmentInformationConverter;
import org.opentcs.kernel.extensions.servicewebapi.v1.converter.PeripheralJobConverter;
import org.opentcs.kernel.extensions.servicewebapi.v1.converter.TransportOrderConverter;
import org.opentcs.kernel.extensions.servicewebapi.v1.converter.VehicleAttachmentInformationConverter;

public class FMSV1RequestHandler
    extends
    V1RequestHandler {

  private static final Set<String> WRITABLE_KEY_PREFIXES = Set.of("fms:", "wes:");

  private final JsonBinder jsonBinder;
  private final KernelExecutorWrapper executorWrapper;
  private final InternalTCSObjectService objectService;
  private final PlantModelXmlValidator plantModelXmlValidator = new PlantModelXmlValidator();

  @Inject
  @SuppressWarnings("checkstyle:ParameterNumber")
  public FMSV1RequestHandler(
      JsonBinder jsonBinder,
      StatusEventDispatcher statusEventDispatcher,
      TransportOrderDispatcherHandler orderDispatcherHandler,
      TransportOrderHandler transportOrderHandler,
      PeripheralJobHandler peripheralJobHandler,
      PeripheralJobDispatcherHandler jobDispatcherHandler,
      PlantModelHandler plantModelHandler,
      EnvironmentalEntityHandler environmentalEntityHandler,
      VehicleHandler vehicleHandler,
      PathHandler pathHandler,
      LocationHandler locationHandler,
      PeripheralHandler peripheralHandler,
      OrderSequenceConverter orderSequenceConverter,
      PeripheralJobConverter peripheralJobConverter,
      TransportOrderConverter transportOrderConverter,
      PeripheralAttachmentInformationConverter peripheralAttachmentInformationConverter,
      VehicleAttachmentInformationConverter vehicleAttachmentInformationConverter,
      LocalKernel kernel,
      @KernelExecutor ScheduledExecutorService kernelExecutor,
      KernelExecutorWrapper executorWrapper,
      InternalTCSObjectService objectService) {
    super(
        jsonBinder,
        statusEventDispatcher,
        orderDispatcherHandler,
        transportOrderHandler,
        peripheralJobHandler,
        jobDispatcherHandler,
        plantModelHandler,
        environmentalEntityHandler,
        vehicleHandler,
        pathHandler,
        locationHandler,
        peripheralHandler,
        orderSequenceConverter,
        peripheralJobConverter,
        transportOrderConverter,
        peripheralAttachmentInformationConverter,
        vehicleAttachmentInformationConverter,
        kernel,
        kernelExecutor);
    this.jsonBinder = requireNonNull(jsonBinder, "jsonBinder");
    this.executorWrapper = requireNonNull(executorWrapper, "executorWrapper");
    this.objectService = requireNonNull(objectService, "objectService");
  }

  @Override
  public EndpointGroup createRoutes() {
    EndpointGroup base = super.createRoutes();
    return () -> {
      base.addEndpoints();
      path("/v1", () -> {
        put("/vehicles/{NAME}/properties", this::handlePutVehicleProperty);
        post("/plantModel/xml/validate", this::handleValidatePlantModelXml);
      });
    };
  }

  private void handlePutVehicleProperty(Context ctx) {
    String vehicleName = ctx.pathParam("NAME");
    String key = writableKey(ctx.queryParam("key"));
    String newValue = ctx.queryParam("newValue");
    ctx.contentType(HttpConstants.CONTENT_TYPE_TEXT_PLAIN_UTF8);
    executorWrapper.callAndWait(() -> {
      Vehicle vehicle = objectService.fetch(Vehicle.class, vehicleName)
          .orElseThrow(() -> new ObjectUnknownException("Unknown vehicle: " + vehicleName));
      objectService.updateObjectProperty(vehicle.getReference(), key, newValue);
    });
    ctx.result("");
  }

  private void handleValidatePlantModelXml(Context ctx) {
    ctx.contentType(HttpConstants.CONTENT_TYPE_APPLICATION_JSON_UTF8);
    ctx.result(jsonBinder.toJson(plantModelXmlValidator.validate(ctx.body())));
  }

  private static String writableKey(String key) {
    if (key == null || key.isBlank()) {
      throw new IllegalArgumentException("Missing query parameter 'key'");
    }
    if (WRITABLE_KEY_PREFIXES.stream().noneMatch(key::startsWith)) {
      throw new IllegalArgumentException(
          "Property key must start with one of " + WRITABLE_KEY_PREFIXES + ": " + key);
    }
    return key;
  }
}
