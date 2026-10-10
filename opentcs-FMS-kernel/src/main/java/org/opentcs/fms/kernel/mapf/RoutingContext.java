package org.opentcs.fms.kernel.mapf;

import java.util.List;
import java.util.Map;
import java.util.Set;

import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.SolverParams;
import org.opentcs.fms.routing.pibt.Footprint;

public final class RoutingContext {

  private static final long SOLVE_DEADLINE_MS = 1_000L;

  private static SolverParams jointParams() {
    return new SolverParams(SolverParams.DEFAULT_WINDOW, SOLVE_DEADLINE_MS, 0);
  }

  private static SolverParams soloParams() {
    return new SolverParams(SolverParams.WINDOW_FULL, SOLVE_DEADLINE_MS, 0);
  }

  private final String modelName;
  private final Roadmap roadmap;
  private final RouteBuilder routeBuilder;
  private final MapfPlanner planner;
  private final Footprint footprint;

  RoutingContext(String modelName, Roadmap roadmap, RouteBuilder routeBuilder, MapfPlanner planner) {
    this.modelName = modelName;
    this.roadmap = roadmap;
    this.routeBuilder = routeBuilder;
    this.planner = planner;
    this.footprint = Footprint.enabled() ? new Footprint(roadmap) : null;
  }

  public String modelName() {
    return modelName;
  }

  public Roadmap roadmap() {
    return roadmap;
  }

  public Footprint footprint() {
    return footprint;
  }

  public RouteBuilder routeBuilder() {
    return routeBuilder;
  }

  public List<String> planSingle(
      String vehicle, String source, String goal, ClaimPolicy claimPolicy,
      List<Footprint.Obstacle> obstacles) {
    List<String> route = new MapfPlanner(roadmap).planRoutes(
        Map.of(vehicle, source), Map.of(vehicle, goal), soloParams(), claimPolicy, obstacles)
        .get(vehicle);
    return route == null ? List.of() : route;
  }

  MapfPlanner.PlanResult planAll(Map<String, String> positions, Map<String, String> goals,
      Set<String> immobileVehicles,
      ClaimPolicy claimPolicy, BodyState bodyState,
      Map<String, List<String>> committedPrefixes, List<Footprint.Obstacle> obstacles) {
    return planner.plan(
        positions, goals, jointParams(), immobileVehicles, claimPolicy, bodyState,
        committedPrefixes, obstacles);
  }
}
