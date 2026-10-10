package org.opentcs.fms.kernel.mapf.adg;

import java.util.List;
import java.util.Map;

public record Adg(
    Map<Step, List<Step>> dependencies,
    Map<String, List<String>> routes,
    Map<String, Integer> coordinatedSteps,
    long generation,
    long builtAtMs) {

  public static final Adg EMPTY = new Adg(Map.of(), Map.of(), Map.of(), 0L, 0L);

  public Adg {
    dependencies = Map.copyOf(dependencies);
    routes = Map.copyOf(routes);
    coordinatedSteps = Map.copyOf(coordinatedSteps);
  }

  public static Adg of(Map<Step, List<Step>> dependencies, Map<String, List<String>> routes) {
    return of(dependencies, routes, Map.of());
  }

  public static Adg of(
      Map<Step, List<Step>> dependencies, Map<String, List<String>> routes,
      Map<String, Integer> coordinatedSteps) {
    return new Adg(dependencies, routes, coordinatedSteps, 0L, 0L);
  }

  public int coordinatedStepOf(String vehicle) {
    return coordinatedSteps.getOrDefault(vehicle, Integer.MAX_VALUE);
  }

  public List<Step> predecessorsOf(Step step) {
    return dependencies.getOrDefault(step, List.of());
  }

  public List<String> routeOf(String vehicle) {
    return routes.getOrDefault(vehicle, List.of());
  }

  public String pointOf(Step step) {
    List<String> route = routeOf(step.vehicle());
    return step.step() >= 0 && step.step() < route.size() ? route.get(step.step()) : null;
  }

  public boolean hasNothingToEnforce() {
    return dependencies.isEmpty();
  }

  public Adg stamped(long generation, long builtAtMs) {
    return new Adg(dependencies, routes, coordinatedSteps, generation, builtAtMs);
  }

  public Adg invalidated() {
    return new Adg(Map.of(), Map.of(), Map.of(), generation + 1, 0L);
  }
}
