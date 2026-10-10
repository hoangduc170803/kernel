package org.opentcs.fms.routing;

/**
 * Tunables for one solver invocation.
 *
 * @param window     planning window {@code w}; the plan is truncated to at most
 *                   {@code window + 1} timesteps. Use {@link #WINDOW_FULL} to
 *                   plan fully to the
 *                   goals (one-shot MAPF).
 * @param deadlineMs wall-clock ceiling for the search, milliseconds; a liveness
 *                   backstop, not the stop condition.
 * @param seed       deterministic RNG seed for PIBT tie-breaking.
 */
public record SolverParams(
    int window,
    long deadlineMs,
    int seed) {

  public static final int WINDOW_FULL = Integer.MAX_VALUE;

  public static final int DEFAULT_WINDOW = configuredWindow();

  private static int configuredWindow() {
    int configured = Integer.getInteger("fms.mapf.window", 10);
    return configured > 0 ? configured : WINDOW_FULL;
  }

  public SolverParams withWindow(int w) {
    return new SolverParams(w, deadlineMs, seed);
  }

  public SolverParams withSeed(int s) {
    return new SolverParams(window, deadlineMs, s);
  }
}
