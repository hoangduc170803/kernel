package org.opentcs.fms.routing;

/**
 * One MAPF problem instance for a single window.
 *
 * <p>A {@code Config} is represented throughout the engine as an {@code int[]}
 * (vertex id per agent); there is no wrapper type, matching the TS
 * {@code Config = VertexId[]}.
 *
 * @param starts current vertex of every agent (the window's starts)
 * @param goals current goal vertex of every agent
 */
public record MapfInstance(int[] starts, int[] goals) {
}
