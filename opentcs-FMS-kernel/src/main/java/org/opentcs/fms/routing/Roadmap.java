package org.opentcs.fms.routing;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.Comparator;
import java.util.Deque;
import java.util.HexFormat;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;

public final class Roadmap {

  public static final int INF = 0x3fffffff;

  public static final int DIR_NONE = -1;
  public static final int DIR_POS_X = 0;
  public static final int DIR_NEG_X = 1;
  public static final int DIR_POS_Y = 2;
  public static final int DIR_NEG_Y = 3;

  public static final int NOSES = 4;

  private static final int NO_STEP = -1;

  private static final int MOVE_COST = 1;
  public static final int QUARTER_TURN_COST = Math.max(
      0, Integer.getInteger("fms.mapf.quarterTurnCost", 1));
  private static final int MAX_QUARTER_TURNS_PER_MOVE = 2;

  public static final int MAX_STEP_WORK = MOVE_COST + MAX_QUARTER_TURNS_PER_MOVE * QUARTER_TURN_COST;

  public static final int AXIS_X = 0;
  public static final int AXIS_Y = 1;

  public static int axisOfDirection(int direction) {
    return direction >> 1;
  }

  public static int opposite(int direction) {
    return direction == DIR_NONE ? DIR_NONE : direction ^ 1;
  }

  public static int lifted(int point, int nose) {
    return point * NOSES + nose;
  }

  public static int distanceAt(int[] turnAware, int vertex, int nose) {
    if (nose != DIR_NONE) {
      return turnAware[lifted(vertex, nose)];
    }
    int best = turnAware[lifted(vertex, 0)];
    for (int h = 1; h < NOSES; h++) {
      best = Math.min(best, turnAware[lifted(vertex, h)]);
    }
    return best;
  }

  public static int turnCostBetween(int noseBefore, int noseAfter) {
    return quarterTurnsBetween(noseBefore, noseAfter) * QUARTER_TURN_COST;
  }

  public static int quarterTurnsBetween(int noseBefore, int noseAfter) {
    if (noseBefore == DIR_NONE || noseAfter == DIR_NONE || noseBefore == noseAfter) {
      return 0;
    }
    return axisOfDirection(noseBefore) == axisOfDirection(noseAfter)
        ? MAX_QUARTER_TURNS_PER_MOVE
        : 1;
  }

  public record Point(String name, double x, double y, double z) {
  }

  public record Path(
      String name, String src, String dst, double length, boolean bidirectional, boolean reverse) {

    public Path(String name, String src, String dst, double length, boolean bidirectional) {
      this(name, src, dst, length, bidirectional, false);
    }
  }

  public record Model(List<Point> points, List<Path> paths, Set<String> cargoSlotPoints,
      Set<String> dropOffPoints, boolean laneDirectionDeclared) {

    public Model(List<Point> points, List<Path> paths) {
      this(points, paths, Set.of(), Set.of(), false);
    }

    public Model(List<Point> points, List<Path> paths, Set<String> cargoSlotPoints) {
      this(points, paths, cargoSlotPoints, Set.of(), false);
    }

    public Model(List<Point> points, List<Path> paths, Set<String> cargoSlotPoints,
        Set<String> dropOffPoints) {
      this(points, paths, cargoSlotPoints, dropOffPoints, false);
    }
  }

  private static final int UNIFORM_WEIGHT = 1;

  private static final int DROP_OFF_LANE_WEIGHT = Math.max(
      UNIFORM_WEIGHT, Integer.getInteger("fms.mapf.dropOffLaneWeight", 4));

  private static final int CARGO_SLOT_WEIGHT_OVERRIDE = Math.max(0, Integer.getInteger("fms.mapf.cargoSlotWeight", 0));

  public static int cargoSlotWeight(int vertexCount) {
    return CARGO_SLOT_WEIGHT_OVERRIDE > 0
        ? CARGO_SLOT_WEIGHT_OVERRIDE
        : weightThatOutranksAnyDetour(vertexCount);
  }

  private static int weightThatOutranksAnyDetour(int vertexCount) {
    return (MOVE_COST + MAX_QUARTER_TURNS_PER_MOVE * QUARTER_TURN_COST) * vertexCount + 1;
  }

  private final String[] names;
  private final double[] xs;
  private final double[] ys;
  private final int[][] neighbors;
  private final int[][] predecessors;
  private final int[] vertexWeights;
  private final Map<String, Integer> indexByName;
  private final Map<Integer, int[]> distCache = new HashMap<>();
  private final Map<Integer, int[]> localDistCache = new HashMap<>();
  private final Map<Integer, int[]> hopCache = new HashMap<>();
  private final Roadmap distanceSource;
  private final BitSet forwardServable;
  private final BitSet dropOffSlots;
  private final int[] dropOffDepth;
  private final int laneAxis;
  private final BitSet locationSlots;
  private final Map<Integer, int[]> turnAwareCache = new HashMap<>();
  private final Map<Integer, int[]> workCache = new HashMap<>();
  private int[][] kinematicContinuations;

  private Roadmap(Roadmap source, int[][] maskedNeighbors, int[][] maskedPredecessors) {
    this.names = source.names;
    this.xs = source.xs;
    this.ys = source.ys;
    this.neighbors = maskedNeighbors;
    this.predecessors = maskedPredecessors;
    this.vertexWeights = source.vertexWeights;
    this.indexByName = source.indexByName;
    this.distanceSource = source.distanceSource;
    this.forwardServable = source.forwardServable;
    this.dropOffSlots = source.dropOffSlots;
    this.dropOffDepth = source.dropOffDepth;
    this.laneAxis = source.laneAxis;
    this.locationSlots = source.locationSlots;
  }

  public Roadmap(Model model) {
    int n = model.points().size();
    this.names = new String[n];
    this.xs = new double[n];
    this.ys = new double[n];
    this.indexByName = new HashMap<>(n * 2);
    for (int i = 0; i < n; i++) {
      Point point = model.points().get(i);
      names[i] = point.name();
      xs[i] = point.x();
      ys[i] = point.y();
      indexByName.put(point.name(), i);
    }

    List<List<Integer>> nbr = new ArrayList<>(n);
    List<List<Integer>> pred = new ArrayList<>(n);
    for (int i = 0; i < n; i++) {
      nbr.add(new ArrayList<>());
      pred.add(new ArrayList<>());
    }
    BitSet servable = new BitSet(n * n);
    for (Path p : model.paths()) {
      Integer a = indexByName.get(p.src());
      Integer b = indexByName.get(p.dst());
      if (a == null || b == null) {
        continue;
      }
      addEdge(nbr, pred, a, b);
      if (!p.reverse()) {
        servable.set(a * n + b);
      }
      if (p.bidirectional()) {
        addEdge(nbr, pred, b, a);
        if (!p.reverse()) {
          servable.set(b * n + a);
        }
      }
    }
    this.forwardServable = servable;

    this.neighbors = toMatrix(nbr);
    this.predecessors = toMatrix(pred);
    Set<String> cargoSlots = model.cargoSlotPoints();
    Set<String> dropOffs = model.dropOffPoints();
    this.vertexWeights = new int[n];
    this.laneAxis = LaneAxis.of(
        model.points(), model.paths(), model.laneDirectionDeclared());
    this.dropOffSlots = new BitSet(n);
    this.locationSlots = new BitSet(n);
    int cargoSlotWeight = cargoSlotWeight(n);
    for (int i = 0; i < n; i++) {
      vertexWeights[i] = cargoSlots.contains(names[i])
          ? cargoSlotWeight
          : dropOffs.contains(names[i]) ? DROP_OFF_LANE_WEIGHT : UNIFORM_WEIGHT;
      if (dropOffs.contains(names[i])) {
        dropOffSlots.set(i);
      }
      if (cargoSlots.contains(names[i]) || dropOffs.contains(names[i])) {
        locationSlots.set(i);
      }
    }
    this.dropOffDepth = deriveDropOffDepth();
    this.distanceSource = this;
  }

  private int[] deriveDropOffDepth() {
    int[] depths = new int[size()];
    Deque<Integer> frontier = new ArrayDeque<>();
    for (int from = 0; from < size(); from++) {
      if (dropOffSlots.get(from)) {
        continue;
      }
      for (int to : neighbors[from]) {
        if (!dropOffSlots.get(to)
            || axisOfDirection(directionOf(from, to)) != laneAxis
            || depths[to] != 0) {
          continue;
        }
        depths[to] = 1;
        frontier.add(to);
      }
    }
    while (!frontier.isEmpty()) {
      int from = frontier.remove();
      for (int to : neighbors[from]) {
        if (dropOffSlots.get(to)
            && axisOfDirection(directionOf(from, to)) == laneAxis
            && depths[to] == 0) {
          depths[to] = depths[from] + 1;
          frontier.add(to);
        }
      }
    }
    return depths;
  }

  public Roadmap blocking(Set<Integer> blockedVertices) {
    if (blockedVertices.isEmpty()) {
      return this;
    }
    int[][] masked = withoutBlockedVertices(neighbors, blockedVertices);
    return new Roadmap(this, masked, transposed(masked));
  }

  private static int[][] withoutBlockedVertices(int[][] adjacency, Set<Integer> blockedVertices) {
    int[][] masked = new int[adjacency.length][];
    for (int v = 0; v < adjacency.length; v++) {
      if (blockedVertices.contains(v)) {
        masked[v] = new int[0];
        continue;
      }
      int kept = 0;
      for (int u : adjacency[v]) {
        if (!blockedVertices.contains(u)) {
          kept++;
        }
      }
      masked[v] = new int[kept];
      int i = 0;
      for (int u : adjacency[v]) {
        if (!blockedVertices.contains(u)) {
          masked[v][i++] = u;
        }
      }
    }
    return masked;
  }

  private static int[][] transposed(int[][] adjacency) {
    int[] inDegree = new int[adjacency.length];
    for (int[] outgoing : adjacency) {
      for (int u : outgoing) {
        inDegree[u]++;
      }
    }
    int[][] incoming = new int[adjacency.length][];
    for (int v = 0; v < adjacency.length; v++) {
      incoming[v] = new int[inDegree[v]];
    }
    int[] filled = new int[adjacency.length];
    for (int v = 0; v < adjacency.length; v++) {
      for (int u : adjacency[v]) {
        incoming[u][filled[u]++] = v;
      }
    }
    return incoming;
  }

  private static void addEdge(List<List<Integer>> nbr, List<List<Integer>> pred, int from, int to) {
    if (from == to || nbr.get(from).contains(to)) {
      return;
    }
    nbr.get(from).add(to);
    pred.get(to).add(from);
  }

  private static int[][] toMatrix(List<List<Integer>> lists) {
    int[][] m = new int[lists.size()][];
    for (int i = 0; i < lists.size(); i++) {
      List<Integer> row = lists.get(i);
      int[] arr = new int[row.size()];
      for (int j = 0; j < arr.length; j++) {
        arr[j] = row.get(j);
      }
      m[i] = arr;
    }
    return m;
  }

  public int size() {
    return names.length;
  }

  public static Map<String, String> effectiveSettings() {
    return Map.of(
        "fms.mapf.quarterTurnCost", Integer.toString(QUARTER_TURN_COST),
        "fms.mapf.dropOffLaneWeight", Integer.toString(DROP_OFF_LANE_WEIGHT),
        "fms.mapf.cargoSlotWeight", Integer.toString(CARGO_SLOT_WEIGHT_OVERRIDE));
  }

  public String contentHash() {
    try {
      MessageDigest digest = MessageDigest.getInstance("SHA-256");
      Integer[] vertices = new Integer[names.length];
      for (int vertex = 0; vertex < names.length; vertex++) {
        vertices[vertex] = vertex;
      }
      Arrays.sort(vertices, Comparator.comparing(vertex -> names[vertex]));
      for (int vertex : vertices) {
        updateDigest(digest, names[vertex]);
        updateDigest(digest, Double.toString(xs[vertex]));
        updateDigest(digest, Double.toString(ys[vertex]));
        updateDigest(digest, Integer.toString(vertexWeights[vertex]));
        String[] adjacent = Arrays.stream(neighbors[vertex])
            .mapToObj(neighbor -> names[neighbor])
            .sorted()
            .toArray(String[]::new);
        updateDigest(digest, String.join(",", adjacent));
      }
      return HexFormat.of().formatHex(digest.digest());
    } catch (NoSuchAlgorithmException e) {
      throw new IllegalStateException(e);
    }
  }

  private static void updateDigest(MessageDigest digest, String value) {
    digest.update(value.getBytes(StandardCharsets.UTF_8));
    digest.update((byte) 0);
  }

  public int[] neighborsOf(int v) {
    return neighbors[v];
  }

  public double xOf(int v) {
    return xs[v];
  }

  public double yOf(int v) {
    return ys[v];
  }

  public int[] predecessorsOf(int v) {
    return predecessors[v];
  }

  public Integer idOf(String name) {
    return indexByName.get(name);
  }

  public String nameOf(int id) {
    return names[id];
  }

  public boolean has(String name) {
    return indexByName.containsKey(name);
  }

  public int[] candidates(int v) {
    int[] nbr = neighbors[v];
    int[] out = new int[nbr.length + 1];
    System.arraycopy(nbr, 0, out, 0, nbr.length);
    out[nbr.length] = v;
    return out;
  }

  public int[] kinematicCandidates(int v, int motion) {
    if (motion == DIR_NONE) {
      return candidates(v);
    }
    if (kinematicContinuations == null) {
      kinematicContinuations = new int[neighbors.length * NOSES][];
    }
    int slot = v * NOSES + motion;
    int[] cached = kinematicContinuations[slot];
    if (cached == null) {
      cached = continuationsOf(v, motion);
      kinematicContinuations[slot] = cached;
    }
    return cached.clone();
  }

  private int[] continuationsOf(int v, int motion) {
    int[] nbr = neighbors[v];
    int[] out = new int[nbr.length + 1];
    int count = 0;
    for (int u : nbr) {
      if (directionOf(v, u) == motion) {
        out[count++] = u;
      }
    }
    out[count++] = v;
    return Arrays.copyOf(out, count);
  }

  public int motionAfter(int from, int to) {
    return from == to ? DIR_NONE : directionOf(from, to);
  }

  public int[] distanceTo(int goal) {
    if (distanceSource != this) {
      return distanceSource.distanceTo(goal);
    }
    int[] cached = distCache.get(goal);
    if (cached != null) {
      return cached;
    }
    int[] dist = weightedDistancesTo(goal);
    distCache.put(goal, dist);
    return dist;
  }

  public int[] localDistanceTo(int goal) {
    if (distanceSource == this) {
      return distanceTo(goal);
    }
    int[] cached = localDistCache.get(goal);
    if (cached != null) {
      return cached;
    }
    int[] dist = weightedDistancesTo(goal);
    localDistCache.put(goal, dist);
    return dist;
  }

  public int[] hopDistanceTo(int goal) {
    if (distanceSource != this) {
      return distanceSource.hopDistanceTo(goal);
    }
    int[] cached = hopCache.get(goal);
    if (cached != null) {
      return cached;
    }
    int[] hops = hopDistancesTo(goal);
    hopCache.put(goal, hops);
    return hops;
  }

  private int[] hopDistancesTo(int goal) {
    int[] hops = new int[size()];
    Arrays.fill(hops, INF);
    hops[goal] = 0;
    Deque<Integer> frontier = new ArrayDeque<>();
    frontier.add(goal);
    while (!frontier.isEmpty()) {
      int v = frontier.poll();
      for (int p : predecessors[v]) {
        if (hops[p] == INF) {
          hops[p] = hops[v] + 1;
          frontier.add(p);
        }
      }
    }
    return hops;
  }

  private int[] weightedDistancesTo(int goal) {
    int[] dist = new int[size()];
    Arrays.fill(dist, INF);
    dist[goal] = 0;
    PriorityQueue<int[]> frontier = new PriorityQueue<>(Comparator.comparingInt(entry -> entry[0]));
    frontier.add(new int[] { 0, goal });
    while (!frontier.isEmpty()) {
      int[] entry = frontier.poll();
      int reached = entry[0];
      int v = entry[1];
      if (reached > dist[v]) {
        continue;
      }
      int throughV = reached + vertexWeights[v];
      for (int p : predecessors[v]) {
        if (dist[p] > throughV) {
          dist[p] = throughV;
          frontier.add(new int[] { throughV, p });
        }
      }
    }
    return dist;
  }

  public boolean sharesLaneWith(int from, int goal) {
    if (from == goal) {
      return true;
    }
    boolean alongY = xs[from] == xs[goal];
    boolean alongX = ys[from] == ys[goal];
    if (!alongX && !alongY) {
      return false;
    }
    int at = from;
    int guard = size();
    while (at != goal && guard-- > 0) {
      int next = NO_STEP;
      for (int neighbour : neighborsOf(at)) {
        if (alongY && xs[neighbour] != xs[goal]) {
          continue;
        }
        if (alongX && ys[neighbour] != ys[goal]) {
          continue;
        }
        if (gapTo(neighbour, goal, alongY) < gapTo(at, goal, alongY)) {
          next = neighbour;
          break;
        }
      }
      if (next == NO_STEP) {
        return false;
      }
      at = next;
    }
    return at == goal;
  }

  private double gapTo(int vertex, int goal, boolean alongY) {
    return alongY ? Math.abs(ys[vertex] - ys[goal]) : Math.abs(xs[vertex] - xs[goal]);
  }

  public boolean isForwardServable(int from, int to) {
    return distanceSource.forwardServable.get(from * distanceSource.names.length + to);
  }

  public int directionOf(int from, int to) {
    if (from == to) {
      return DIR_NONE;
    }
    double dx = xs[to] - xs[from];
    double dy = ys[to] - ys[from];
    if (Math.abs(dx) == Math.abs(dy)) {
      return DIR_NONE;
    }
    if (Math.abs(dx) > Math.abs(dy)) {
      return dx > 0 ? DIR_POS_X : DIR_NEG_X;
    }
    return dy > 0 ? DIR_POS_Y : DIR_NEG_Y;
  }

  public int requiredNose(int from, int to) {
    int travel = directionOf(from, to);
    if (travel == DIR_NONE) {
      return DIR_NONE;
    }
    return isForwardServable(from, to) ? travel : opposite(travel);
  }

  public int noseAfter(int noseBefore, int from, int to) {
    int required = requiredNose(from, to);
    return required == DIR_NONE ? noseBefore : required;
  }

  public int[] turnAwareDistanceTo(int goal) {
    if (distanceSource != this) {
      return distanceSource.turnAwareDistanceTo(goal);
    }
    int[] cached = turnAwareCache.get(goal);
    if (cached != null) {
      return cached;
    }
    int[] dist = turnAwareDistancesTo(goal, true);
    turnAwareCache.put(goal, dist);
    return dist;
  }

  public int[] workDistanceTo(int goal) {
    if (distanceSource != this) {
      return distanceSource.workDistanceTo(goal);
    }
    int[] cached = workCache.get(goal);
    if (cached != null) {
      return cached;
    }
    int[] dist = turnAwareDistancesTo(goal, false);
    workCache.put(goal, dist);
    return dist;
  }

  private int[] turnAwareDistancesTo(int goal, boolean chargeVertexWeight) {
    int[] dist = new int[size() * NOSES];
    Arrays.fill(dist, INF);
    PriorityQueue<int[]> frontier = new PriorityQueue<>(Comparator.comparingInt(e -> e[0]));
    for (int nose = 0; nose < NOSES; nose++) {
      dist[lifted(goal, nose)] = 0;
      frontier.add(new int[] { 0, lifted(goal, nose) });
    }
    while (!frontier.isEmpty()) {
      int[] entry = frontier.poll();
      int reached = entry[0];
      int state = entry[1];
      if (reached > dist[state]) {
        continue;
      }
      int point = state / NOSES;
      int nose = state % NOSES;
      for (int turned = 0; turned < NOSES; turned++) {
        if (axisOfDirection(turned) != axisOfDirection(nose)) {
          relax(dist, frontier, lifted(point, turned), reached + QUARTER_TURN_COST);
        }
      }
      for (int predecessor : predecessorsOf(point)) {
        if (requiredNose(predecessor, point) != nose) {
          continue;
        }
        relax(dist, frontier, lifted(predecessor, nose),
            reached + (chargeVertexWeight ? weightOf(point) : MOVE_COST));
      }
    }
    return dist;
  }

  private static void relax(int[] dist, PriorityQueue<int[]> frontier, int state, int cost) {
    if (dist[state] > cost) {
      dist[state] = cost;
      frontier.add(new int[] { cost, state });
    }
  }

  public boolean crossesLane(int from, int to) {
    int direction = directionOf(from, to);
    return direction != DIR_NONE && axisOfDirection(direction) != distanceSource.laneAxis;
  }

  public boolean isCargoSlot(int vertex) {
    return distanceSource.locationSlots.get(vertex);
  }

  public boolean isDropOffSlot(int vertex) {
    return distanceSource.dropOffSlots.get(vertex);
  }

  public int dropOffDepth(int vertex) {
    return distanceSource.dropOffDepth[vertex];
  }

  public int weightOf(int vertex) {
    return distanceSource.vertexWeights[vertex];
  }

  public int maxVertexWeight() {
    int max = 1;
    for (int w : distanceSource.vertexWeights) {
      max = Math.max(max, w);
    }
    return max;
  }

  public int distance(int from, int goal) {
    return distanceTo(goal)[from];
  }
}
