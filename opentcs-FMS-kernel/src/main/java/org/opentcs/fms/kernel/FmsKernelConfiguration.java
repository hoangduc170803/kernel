package org.opentcs.fms.kernel;

import org.opentcs.configuration.ConfigurationEntry;
import org.opentcs.configuration.ConfigurationPrefix;

@ConfigurationPrefix(FmsKernelConfiguration.PREFIX)
public interface FmsKernelConfiguration {

  String PREFIX = "fmskernel";

  String MAPF_ENABLED = "fms.mapfEnabled";

  @ConfigurationEntry(
      type = "Boolean",
      description = {
          "Whether the FMS multi-agent path finding layer is active.",
          "When set to false, the kernel uses the stock openTCS router, the ADG precedence"
              + " scheduler module is not registered, and the dispatcher does not run MAPF solves."
      },
      changesApplied = ConfigurationEntry.ChangesApplied.ON_APPLICATION_START,
      orderKey = "0_mapf"
  )
  boolean mapfEnabled();
}
