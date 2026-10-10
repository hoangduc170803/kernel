package org.opentcs.fms.kernel.mapf.adg;

import java.util.List;
import java.util.Map;

import jakarta.inject.Singleton;

import org.opentcs.fms.kernel.mapf.BodyState;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.pibt.Footprint;

@Singleton
public final class AdgService {

  private volatile Adg adg = Adg.EMPTY;

  public Adg adg() {
    return adg;
  }

  public Adg install(
      Map<String, List<String>> spaceTimePaths, int coordinatedTicks, Roadmap roadmap,
      Footprint footprint, BodyState bodyState, long builtAtMs) {
    Adg replaced = adg;
    adg = AdgBuilder.build(spaceTimePaths, coordinatedTicks, roadmap, footprint, bodyState)
        .stamped(replaced.generation() + 1, builtAtMs);
    return replaced;
  }

  public void invalidate() {
    adg = adg.invalidated();
  }

  public void touch(long builtAtMs) {
    adg = adg.stamped(adg.generation(), builtAtMs);
  }
}
