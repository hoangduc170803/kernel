package org.opentcs.fms.routing.pibt;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import org.opentcs.fms.routing.Roadmap;

public final class Footprint {

  private static final org.slf4j.Logger logger = org.slf4j.LoggerFactory.getLogger(Footprint.class);

  private static final AtomicBoolean MODEL_LOGGED = new AtomicBoolean();

  private static final int[] NOSES_ALONG_EACH_AXIS = { Roadmap.DIR_POS_X, Roadmap.DIR_POS_Y };

  private static final int AXIS_NONE = -1;
  private static final int AXIS_X = 0;
  private static final int AXIS_Y = 1;

  private static final String ENABLED_PROPERTY = "fms.pibt.footprint";

  private static final double VEHICLE_LENGTH_MM = Double
      .parseDouble(System.getProperty("fms.pibt.vehicleLengthMm", "840"));

  private static final double VEHICLE_WIDTH_MM = Double
      .parseDouble(System.getProperty("fms.pibt.vehicleWidthMm", "425"));

  private static final double CARGO_LENGTH_MM = Double
      .parseDouble(System.getProperty("fms.pibt.cargoLengthMm", "800"));

  private static final double CARGO_WIDTH_MM = Double
      .parseDouble(System.getProperty("fms.pibt.cargoWidthMm", "800"));

  private static final double CLEARANCE_MM = Double
      .parseDouble(System.getProperty("fms.pibt.clearanceMm", "0"));

  public static boolean enabled() {
    return Boolean.parseBoolean(System.getProperty(ENABLED_PROPERTY, "true"));
  }

  public static Map<String, String> effectiveSettings() {
    return Map.of(
        "fms.pibt.footprint", String.valueOf(enabled()),
        "fms.pibt.vehicleLengthMm", Double.toString(VEHICLE_LENGTH_MM),
        "fms.pibt.vehicleWidthMm", Double.toString(VEHICLE_WIDTH_MM),
        "fms.pibt.cargoLengthMm", Double.toString(CARGO_LENGTH_MM),
        "fms.pibt.cargoWidthMm", Double.toString(CARGO_WIDTH_MM),
        "fms.pibt.clearanceMm", Double.toString(CLEARANCE_MM));
  }

  public record Motion(int from, int to, int nose, boolean loaded) {

    public static Motion resting(int at, int nose, boolean loaded) {
      return new Motion(at, at, nose, loaded);
    }

    public boolean moves() {
      return from != to;
    }

    boolean turns(Footprint footprint) {
      return footprint.isRotation(nose, from, to);
    }
  }

  public record BodyPose(int at, int nose, boolean loaded, boolean turning) {
  }

  public record Obstacle(String name, double minX, double maxX, double minY, double maxY) {

    public static Obstacle bounding(String name, double[] xs, double[] ys) {
      return new Obstacle(
          name,
          Arrays.stream(xs).min().orElseThrow(), Arrays.stream(xs).max().orElseThrow(),
          Arrays.stream(ys).min().orElseThrow(), Arrays.stream(ys).max().orElseThrow());
    }

    double centreX() {
      return (minX + maxX) / 2.0;
    }

    double centreY() {
      return (minY + maxY) / 2.0;
    }
  }

  private final Roadmap roadmap;
  private final double sweptRadiusEmpty;
  private final double sweptRadiusLoaded;
  private final double alongHalfEmpty;
  private final double acrossHalfEmpty;
  private final double alongHalfLoaded;
  private final double acrossHalfLoaded;
  private final double[] interactRadiusSq;
  private final int[][] reachIndex;
  private final int[][] exclusionIndex;
  private final int[][] turnExclusionIndex;

  public Footprint(Roadmap roadmap) {
    this.roadmap = roadmap;
    this.reachIndex = new int[roadmap.size()][];
    this.exclusionIndex = new int[roadmap.size()][];
    this.turnExclusionIndex = new int[roadmap.size()][];
    this.alongHalfEmpty = VEHICLE_LENGTH_MM / 2.0;
    this.acrossHalfEmpty = VEHICLE_WIDTH_MM / 2.0;
    this.alongHalfLoaded = Math.max(VEHICLE_LENGTH_MM, CARGO_LENGTH_MM) / 2.0;
    this.acrossHalfLoaded = Math.max(VEHICLE_WIDTH_MM, CARGO_WIDTH_MM) / 2.0;
    this.sweptRadiusEmpty = Math.hypot(alongHalfEmpty, acrossHalfEmpty);
    this.sweptRadiusLoaded = Math.hypot(alongHalfLoaded, acrossHalfLoaded);
    this.interactRadiusSq = interactRadiiSquared();
    logModelOnce();
  }

  private void warnIfCoordinatesLookUnitless() {
    double median = medianEdgeLength();
    if (median <= 0 || median >= VEHICLE_WIDTH_MM) {
      return;
    }
    logger.warn(
        "[MAPF footprint] median edge length is {}mm, far narrower than the {}mm vehicle:"
            + " these roadmap coordinates look like graph labels rather than millimetres."
            + " Every configuration will be reported as overlapping and the solver will"
            + " return no plan.",
        round(median), VEHICLE_WIDTH_MM);
  }

  private double medianEdgeLength() {
    List<Double> lengths = new ArrayList<>();
    for (int vertex = 0; vertex < roadmap.size(); vertex++) {
      for (int neighbour : roadmap.neighborsOf(vertex)) {
        lengths.add(Math.hypot(
            roadmap.xOf(neighbour) - roadmap.xOf(vertex),
            roadmap.yOf(neighbour) - roadmap.yOf(vertex)));
      }
    }
    if (lengths.isEmpty()) {
      return 0;
    }
    Collections.sort(lengths);
    return lengths.get(lengths.size() / 2);
  }

  private void logModelOnce() {
    if (!MODEL_LOGGED.compareAndSet(false, true)) {
      return;
    }
    warnIfCoordinatesLookUnitless();
    logger.info(
        "[MAPF footprint] enabled={} vehicle={}x{}mm cargo={}x{}mm clearance={}mm"
            + " | swept radius empty={} loaded={} | half-extent along/across empty={}/{}"
            + " loaded={}/{} | two loaded turns need {}mm apart, a loaded turn beside a"
            + " resting long side needs {}mm",
        enabled(), VEHICLE_LENGTH_MM, VEHICLE_WIDTH_MM, CARGO_LENGTH_MM, CARGO_WIDTH_MM,
        CLEARANCE_MM, round(sweptRadiusEmpty), round(sweptRadiusLoaded), round(alongHalfEmpty),
        round(acrossHalfEmpty), round(alongHalfLoaded), round(acrossHalfLoaded),
        round(2 * sweptRadiusLoaded + CLEARANCE_MM),
        round(sweptRadiusLoaded + alongHalfLoaded + CLEARANCE_MM));
  }

  private static double round(double value) {
    return Math.round(value * 10.0) / 10.0;
  }

  private double[] interactRadiiSquared() {
    double[] longestEdgeFrom = new double[roadmap.size()];
    double longestEdge = 0;
    for (int vertex = 0; vertex < roadmap.size(); vertex++) {
      for (int neighbour : roadmap.neighborsOf(vertex)) {
        double length = Math.hypot(
            roadmap.xOf(neighbour) - roadmap.xOf(vertex),
            roadmap.yOf(neighbour) - roadmap.yOf(vertex));
        longestEdgeFrom[vertex] = Math.max(longestEdgeFrom[vertex], length);
        longestEdge = Math.max(longestEdge, length);
      }
    }
    double[] squared = new double[roadmap.size()];
    for (int vertex = 0; vertex < roadmap.size(); vertex++) {
      double reach = maxReach() + longestEdgeFrom[vertex] + longestEdge;
      squared[vertex] = reach * reach;
    }
    return squared;
  }

  public boolean mayInteract(int ego, int other) {
    double dx = roadmap.xOf(ego) - roadmap.xOf(other);
    double dy = roadmap.yOf(ego) - roadmap.yOf(other);
    return dx * dx + dy * dy < interactRadiusSq[ego];
  }

  public int[] withinReach(int vertex) {
    int[] cached = reachIndex[vertex];
    if (cached != null) {
      return cached;
    }
    int[] scratch = new int[roadmap.size()];
    int found = 0;
    for (int other = 0; other < roadmap.size(); other++) {
      if (other != vertex && mayInteract(vertex, other)) {
        scratch[found++] = other;
      }
    }
    int[] neighbourhood = Arrays.copyOf(scratch, found);
    reachIndex[vertex] = neighbourhood;
    return neighbourhood;
  }

  public int[] cannotCoexistWith(int vertex) {
    int[] cached = exclusionIndex[vertex];
    if (cached != null) {
      return cached;
    }
    int[] scratch = new int[roadmap.size()];
    int found = 0;
    for (int other : withinReach(vertex)) {
      if (noHeadingFits(vertex, true, other, true)) {
        scratch[found++] = other;
      }
    }
    int[] exclusive = Arrays.copyOf(scratch, found);
    exclusionIndex[vertex] = exclusive;
    return exclusive;
  }

  public int[] cannotCoexistWhileTurningAt(int vertex) {
    int[] cached = turnExclusionIndex[vertex];
    if (cached != null) {
      return cached;
    }
    int[] scratch = new int[roadmap.size()];
    int found = 0;
    for (int other : withinReach(vertex)) {
      if (noHeadingEscapesTheTurn(vertex, other)) {
        scratch[found++] = other;
      }
    }
    int[] exclusive = Arrays.copyOf(scratch, found);
    turnExclusionIndex[vertex] = exclusive;
    return exclusive;
  }

  private boolean noHeadingEscapesTheTurn(int pivot, int cell) {
    for (int nose : NOSES_ALONG_EACH_AXIS) {
      if (!turningBodyClashes(pivot, true, cell, nose, true, false)) {
        return false;
      }
    }
    return true;
  }

  public double sweptRadius(boolean loaded) {
    return loaded ? sweptRadiusLoaded : sweptRadiusEmpty;
  }

  public double maxReach() {
    return 2 * sweptRadiusLoaded + CLEARANCE_MM;
  }

  private int axisOf(int from, int to) {
    if (from == to) {
      return AXIS_NONE;
    }
    double dx = Math.abs(roadmap.xOf(to) - roadmap.xOf(from));
    double dy = Math.abs(roadmap.yOf(to) - roadmap.yOf(from));
    if (dx == dy) {
      return AXIS_NONE;
    }
    return dx > dy ? AXIS_X : AXIS_Y;
  }

  public boolean isRotation(int noseBefore, int from, int to) {
    int noseAfter = noseAfter(noseBefore, from, to);
    if (noseBefore == Roadmap.DIR_NONE || noseAfter == Roadmap.DIR_NONE
        || noseBefore == noseAfter) {
      return false;
    }
    return Roadmap.axisOfDirection(noseBefore) != Roadmap.axisOfDirection(noseAfter);
  }

  public int noseAfter(int noseBefore, int from, int to) {
    return roadmap.noseAfter(noseBefore, from, to);
  }

  /**
   * Collision is evaluated at node configurations only: the resting bodies at
   * each agent's
   * destination cell, plus the swept disc at the cell where an agent pivots. The
   * straight-line
   * path between two cells is deliberately not checked.
   *
   * <p>
   * Consequence: two vehicles that move at the same time and share no vertex can
   * pass through
   * each other between timesteps. On v7-map-xuong.xml there are 608 such edge
   * pairs, every one of
   * them needing both vehicles loaded. Worked example, both loaded and nosed
   * along X, so each body
   * is 840mm along X and 800mm across:
   *
   * <pre>
   *   A: 0001 (25400,-3050) -&gt; 0002 (26350,-3050)   heading east
   *   B: 0007 (26350,-2300) -&gt; 0006 (25400,-2300)   heading west, one row over
   *
   *   |dy| stays 750 and two loaded bodies need 800 across, so they overlap in Y all step.
   *   |dx| runs 950 -&gt; 0 -&gt; 950 while they need 840 apart:
   *
   *     t=0.00  |dx|=950  clear      &lt;- node configuration, checked
   *     t=0.50  |dx|=  0  OVERLAP    &lt;- no node here, not checked
   *     t=1.00  |dx|=950  clear      &lt;- node configuration, checked
   * </pre>
   *
   * <p>
   * Aisles 750mm apart cannot let two 800mm-wide loads pass, so this is a real
   * collision the
   * planner will not prevent. The vehicles' own safety scanners are the remaining
   * defence.
   */
  public boolean collideMoving(Motion ego, Motion other) {
    boolean egoRotates = ego.turns(this);
    boolean otherRotates = other.turns(this);

    if (egoRotates) {
      if (turningBodyClashes(ego.from(), ego.loaded(), other.from(), other.nose(), other.loaded(),
          otherRotates)) {
        return true;
      }
      boolean otherFollowsEgoOut = other.to() == ego.from();
      if (other.moves() && !otherFollowsEgoOut
          && turningBodyClashes(ego.from(), ego.loaded(), other.to(), other.nose(), other.loaded(),
              otherRotates)) {
        return true;
      }
    }
    if (otherRotates) {
      if (turningBodyClashes(other.from(), other.loaded(), ego.from(), ego.nose(), ego.loaded(),
          egoRotates)) {
        return true;
      }
      boolean egoFollowsOtherOut = ego.to() == other.from();
      if (ego.moves() && !egoFollowsOtherOut
          && turningBodyClashes(other.from(), other.loaded(), ego.to(), ego.nose(), ego.loaded(),
              egoRotates)) {
        return true;
      }
    }
    return restingTooClose(
        ego.to(), noseAfter(ego.nose(), ego.from(), ego.to()), ego.loaded(),
        other.to(), noseAfter(other.nose(), other.from(), other.to()), other.loaded());
  }

  public boolean mayInteract(int ego, Obstacle obstacle) {
    return distanceSquaredToObstacle(ego, obstacle) < interactRadiusSq[ego];
  }

  public boolean collideMoving(Motion ego, Obstacle obstacle) {
    if (ego.turns(this) && turningBodyReaches(ego.from(), ego.loaded(), obstacle)) {
      return true;
    }
    int noseAtEnd = noseAfter(ego.nose(), ego.from(), ego.to());
    if (!restingBodyOverlaps(ego.to(), noseAtEnd, ego.loaded(), obstacle)) {
      return false;
    }
    boolean alreadyOverlapping
        = restingBodyOverlaps(ego.from(), ego.nose(), ego.loaded(), obstacle);
    return !alreadyOverlapping || movesCloserTo(obstacle, ego.from(), ego.to());
  }

  private boolean turningBodyReaches(int pivot, boolean loaded, Obstacle obstacle) {
    double reach = sweptRadius(loaded) + CLEARANCE_MM;
    return distanceSquaredToObstacle(pivot, obstacle) < reach * reach;
  }

  private boolean restingBodyOverlaps(int at, int nose, boolean loaded, Obstacle obstacle) {
    double halfX = halfAlongX(nose, loaded) + CLEARANCE_MM;
    double halfY = halfAlongY(nose, loaded) + CLEARANCE_MM;
    double x = roadmap.xOf(at);
    double y = roadmap.yOf(at);
    return x + halfX > obstacle.minX() && x - halfX < obstacle.maxX()
        && y + halfY > obstacle.minY() && y - halfY < obstacle.maxY();
  }

  private boolean movesCloserTo(Obstacle obstacle, int from, int to) {
    return distanceSquaredToCentre(to, obstacle) < distanceSquaredToCentre(from, obstacle);
  }

  private double distanceSquaredToCentre(int vertex, Obstacle obstacle) {
    double dx = roadmap.xOf(vertex) - obstacle.centreX();
    double dy = roadmap.yOf(vertex) - obstacle.centreY();
    return dx * dx + dy * dy;
  }

  private double distanceSquaredToObstacle(int vertex, Obstacle obstacle) {
    double x = roadmap.xOf(vertex);
    double y = roadmap.yOf(vertex);
    double dx = Math.max(Math.max(obstacle.minX() - x, 0), x - obstacle.maxX());
    double dy = Math.max(Math.max(obstacle.minY() - y, 0), y - obstacle.maxY());
    return dx * dx + dy * dy;
  }

  public boolean noHeadingFits(int at, boolean loaded, int otherAt, boolean otherLoaded) {
    for (int nose : NOSES_ALONG_EACH_AXIS) {
      for (int otherNose : NOSES_ALONG_EACH_AXIS) {
        if (!restingTooClose(at, nose, loaded, otherAt, otherNose, otherLoaded)) {
          return false;
        }
      }
    }
    return true;
  }

  public boolean bodiesClash(
      int at, int nose, boolean loaded, int otherAt, int otherNose, boolean otherLoaded) {
    return restingTooClose(at, nose, loaded, otherAt, otherNose, otherLoaded);
  }

  public boolean bodiesClash(BodyPose ego, BodyPose other) {
    if (ego.turning()) {
      return turningBodyClashes(ego.at(), ego.loaded(),
          other.at(), other.nose(), other.loaded(), other.turning());
    }
    if (other.turning()) {
      return turningBodyClashes(other.at(), other.loaded(),
          ego.at(), ego.nose(), ego.loaded(), false);
    }
    return bodiesClash(ego.at(), ego.nose(), ego.loaded(),
        other.at(), other.nose(), other.loaded());
  }

  private boolean turningBodyClashes(
      int at, boolean loaded, int otherAt, int otherNose, boolean otherLoaded,
      boolean otherTurning) {
    if (at == otherAt) {
      return true;
    }
    double reach = sweptRadius(loaded);
    double extent = otherTurning
        ? sweptRadius(otherLoaded)
        : halfExtentToward(Roadmap.axisOfDirection(otherNose), axisOf(at, otherAt), otherLoaded);
    return closerThan(at, otherAt, reach + extent + CLEARANCE_MM);
  }

  private boolean restingTooClose(
      int a, int noseA, boolean loadedA, int b, int noseB, boolean loadedB) {
    if (a == b) {
      return true;
    }
    double dx = Math.abs(roadmap.xOf(a) - roadmap.xOf(b));
    double dy = Math.abs(roadmap.yOf(a) - roadmap.yOf(b));
    return dx < halfAlongX(noseA, loadedA) + halfAlongX(noseB, loadedB) + CLEARANCE_MM
        && dy < halfAlongY(noseA, loadedA) + halfAlongY(noseB, loadedB) + CLEARANCE_MM;
  }

  private double halfAlongX(int nose, boolean loaded) {
    return halfExtentToward(Roadmap.axisOfDirection(nose), AXIS_X, loaded);
  }

  private double halfAlongY(int nose, boolean loaded) {
    return halfExtentToward(Roadmap.axisOfDirection(nose), AXIS_Y, loaded);
  }

  private boolean closerThan(int a, int b, double separation) {
    double dx = roadmap.xOf(a) - roadmap.xOf(b);
    double dy = roadmap.yOf(a) - roadmap.yOf(b);
    return dx * dx + dy * dy < separation * separation;
  }

  private double halfExtentToward(int bodyAxis, int towardAxis, boolean loaded) {
    double along = loaded ? alongHalfLoaded : alongHalfEmpty;
    double across = loaded ? acrossHalfLoaded : acrossHalfEmpty;
    if (bodyAxis == AXIS_NONE || towardAxis == AXIS_NONE) {
      return Math.max(along, across);
    }
    return bodyAxis == towardAxis ? along : across;
  }
}
