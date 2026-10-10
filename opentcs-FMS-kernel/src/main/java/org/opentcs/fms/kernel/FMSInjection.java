package org.opentcs.fms.kernel;

import org.opentcs.customizations.kernel.KernelInjectionModule;
import org.opentcs.fms.kernel.mapf.MapfCoordinator;
import org.opentcs.fms.kernel.mapf.MapfPrecedenceModule;
import org.opentcs.fms.kernel.mapf.adg.AdgService;
import org.opentcs.fms.kernel.rerouting.FmsRegularDriveOrderMerger;
import org.opentcs.fms.kernel.scheduling.FMSScheduler;
import org.opentcs.fms.kernel.transformers.LaserActionTransformerFactory;
import org.opentcs.fms.kernel.vda5050.AubotCommAdapterFactory;
import org.opentcs.fms.kernel.vda5050.LostNavigationListener;
import org.opentcs.strategies.basic.dispatching.rerouting.RegularDriveOrderMerger;
import org.opentcs.strategies.basic.dispatching.selection.vehicles.IsAvailableForAnyOrder;

import org.opentcs.kernel.extensions.servicewebapi.v1.V1RequestHandler;

import com.google.inject.Scopes;
import com.google.inject.name.Names;

public class FMSInjection extends KernelInjectionModule {
        private static final org.slf4j.Logger logger = org.slf4j.LoggerFactory.getLogger(FMSInjection.class);

        @Override
        protected void configure() {
                super.configure();
                boolean mapfEnabled = readMapfEnabled();
                logger.info(
                                "FMSInjection: binding MapfCoordinator + FMSDispatcher + FMSScheduler "
                                                + "+ loop-aware RegularDriveOrderMerger + LaserActionTransformerFactory"
                                                + " (mapfEnabled={})",
                                mapfEnabled);
                bindConstant().annotatedWith(Names.named(FmsKernelConfiguration.MAPF_ENABLED))
                                .to(mapfEnabled);
                bind(AdgService.class).in(Scopes.SINGLETON);
                bind(FleetSnapshotService.class).in(Scopes.SINGLETON);
                bind(MapfCoordinator.class).in(Scopes.SINGLETON);
                if (mapfEnabled) {
                        logger.info("FMSInjection: binding FMSRouter + MapfPrecedenceModule");
                        bindRouter(FMSRouter.class);
                        schedulerModuleBinder().addBinding().to(MapfPrecedenceModule.class);
                } else {
                        logger.info(
                                        "FMSInjection: {}=false, leaving the stock openTCS router in place and skipping"
                                                        + " the ADG precedence module",
                                        FmsKernelConfiguration.PREFIX + ".mapfEnabled");
                }
                bindDispatcher(FMSDispatcher.class);
                bind(IsAvailableForAnyOrder.class).to(FmsIsAvailableForAnyOrder.class).in(Scopes.SINGLETON);
                bindScheduler(FMSScheduler.class);
                bind(RegularDriveOrderMerger.class).to(FmsRegularDriveOrderMerger.class).in(Scopes.SINGLETON);
                bind(V1RequestHandler.class).to(FMSV1RequestHandler.class).in(Scopes.SINGLETON);
                extensionsBinderOperating().addBinding().to(WaitDropOffListener.class)
                                .in(Scopes.SINGLETON);
                extensionsBinderOperating().addBinding().to(LostNavigationListener.class)
                                .in(Scopes.SINGLETON);
                extensionsBinderOperating().addBinding().to(VehicleMoveListener.class)
                                .in(Scopes.SINGLETON);
                vehicleDataTransformersBinder().addBinding().to(LaserActionTransformerFactory.class);
                logger.info("bind LaserActionTransformerFactory done");
                vehicleCommAdaptersBinder().addBinding().to(AubotCommAdapterFactory.class);
        }

        private boolean readMapfEnabled() {
                try {
                        return getConfigBindingProvider()
                                        .get(FmsKernelConfiguration.PREFIX, FmsKernelConfiguration.class)
                                        .mapfEnabled();
                } catch (Exception e) {
                        logger.info(
                                        "FMSInjection: no {}.mapfEnabled in the configuration, defaulting to true ({})",
                                        FmsKernelConfiguration.PREFIX, e.toString());
                        return true;
                }
        }
}
