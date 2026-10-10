package org.opentcs.fms.kernel.mapf;

import static org.assertj.core.api.Assertions.assertThat;
import static org.opentcs.fms.kernel.mapf.RoadmapFixtures.gridRoadmap;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.opentcs.fms.routing.Roadmap;
import org.opentcs.fms.routing.SolverParams;

class MapfPlannerTest {

    @BeforeAll
    static void theseFixturesMeasureInGraphUnitsNotMillimetres() {
        System.setProperty("fms.pibt.footprint", "false");
    }

    @AfterAll
    static void restoreFootprintForEveryoneElse() {
        System.clearProperty("fms.pibt.footprint");
    }

    // --- planRoutes(): end-to-end on a grid -------------------------------------

    @Test
    void producesSimpleSpatialRoutesToGoals() {
        Roadmap r = gridRoadmap(4, 4);
        MapfPlanner planner = new MapfPlanner(r);

        Map<String, List<String>> routes = planner.planRoutes(
                Map.of("V1", "r0c0", "V2", "r0c3"),
                Map.of("V1", "r3c3", "V2", "r3c0"),
                new SolverParams(SolverParams.WINDOW_FULL, 1000, 1),
                ClaimPolicy.DEFAULT);

        assertThat(routes).containsOnlyKeys("V1", "V2");
        assertRouteValid(r, routes.get("V1"), "r0c0", "r3c3");
        assertRouteValid(r, routes.get("V2"), "r0c3", "r3c0");
    }

    @Test
    void skipsVehiclesWithUnknownPositionAndStaysWhenNoGoal() {
        Roadmap r = gridRoadmap(3, 3);
        MapfPlanner planner = new MapfPlanner(r);

        HashMap<String, String> goals = new HashMap<>();
        goals.put("V1", null); // no goal -> stay
        Map<String, List<String>> routes = planner.planRoutes(
                Map.of("V1", "r1c1", "Ghost", "nowhere"),
                goals,
                new SolverParams(SolverParams.WINDOW_FULL, 1000, 0),
                ClaimPolicy.DEFAULT);

        assertThat(routes).containsOnlyKeys("V1"); // Ghost skipped
        assertThat(routes.get("V1")).containsExactly("r1c1"); // stays put
    }

    @Test
    void aVehicleWhoseGoalIsHeldByAnotherStaysWhereItIs() {
        Roadmap r = gridRoadmap(3, 3);

        Map<String, List<String>> routes = new MapfPlanner(r).planRoutes(
                Map.of("V1", "r0c0", "V2", "r2c2"),
                Map.of("V1", "r2c2", "V2", "r2c2"),
                new SolverParams(SolverParams.WINDOW_FULL, 1000, 1),
                ClaimPolicy.DEFAULT);

        assertThat(routes.get("V2")).containsExactly("r2c2");
        assertThat(routes.get("V1"))
                .as("goals come from WES alone now; with r2c2 walled off by the vehicle standing on it"
                        + " there is nothing to substitute, so V1 holds its point and the block is reported"
                        + " instead of turning into a route towards a cell it cannot have")
                .containsExactly("r0c0");
    }

    private static void assertRouteValid(Roadmap r, List<String> route, String start, String goal) {
        assertThat(route).startsWith(start);
        assertThat(route.get(route.size() - 1)).isEqualTo(goal);
        // Revisits (yield excursions) are LEGAL driven routes since 2026-07-08; the
        // remaining invariants are: no waits (consecutive points differ) ...
        for (int i = 0; i + 1 < route.size(); i++) {
            assertThat(route.get(i)).isNotEqualTo(route.get(i + 1));
        }
        // ... and every consecutive pair is a real edge:
        for (int i = 0; i + 1 < route.size(); i++) {
            assertThat(r.distance(r.idOf(route.get(i)), r.idOf(route.get(i + 1))))
                    .as("consecutive points must be a real edge: %s -> %s", route.get(i), route.get(i + 1))
                    .isEqualTo(1);
        }
    }

    private static int seedCell(Roadmap map, int row, int col) {
        return map.idOf("r" + row + "c" + col);
    }

    private static int motionSeed(Roadmap map, int[] plan, int start) {
        return MapfPlanner.motionEnteringTheCurrentCell(
                map, MapfPlanner.planFromWhereTheVehicleActuallyIs(plan, start), start);
    }

    @Test
    void aVehicleThatRolledIntoItsCellIsStillRolling() {
        Roadmap map = gridRoadmap(4, 4);
        int[] plan = {
                seedCell(map, 0, 0), seedCell(map, 0, 1), seedCell(map, 0, 2), seedCell(map, 0, 3) };

        assertThat(motionSeed(map, plan, seedCell(map, 0, 2)))
                .isEqualTo(map.directionOf(seedCell(map, 0, 1), seedCell(map, 0, 2)));
    }

    @Test
    void aWaitTickReadsAsStopped() {
        Roadmap map = gridRoadmap(4, 4);
        int[] plan = {
                seedCell(map, 0, 0), seedCell(map, 0, 1), seedCell(map, 0, 1), seedCell(map, 1, 1) };

        assertThat(motionSeed(map, plan, seedCell(map, 0, 1)))
                .as("the plan stands still on r0c1, so the vehicle has already braked and may turn")
                .isEqualTo(Roadmap.DIR_NONE);
    }

    @Test
    void aRevisitedCellTakesTheOccasionTheWarmStartTook() {
        Roadmap map = gridRoadmap(4, 4);
        int[] plan = {
                seedCell(map, 0, 0), seedCell(map, 0, 1), seedCell(map, 0, 2), seedCell(map, 0, 1),
                seedCell(map, 1, 1) };
        int[] carried = MapfPlanner.planFromWhereTheVehicleActuallyIs(plan, seedCell(map, 0, 1));

        assertThat(carried[1])
                .as("whichever occasion the warm start picks, the motion seed must be read off the same"
                        + " one, or the two seeds describe different vehicles")
                .isEqualTo(seedCell(map, 0, 1));
        assertThat(MapfPlanner.motionEnteringTheCurrentCell(map, carried, seedCell(map, 0, 1)))
                .isEqualTo(map.directionOf(carried[0], carried[1]));
    }

    @Test
    void anUnusableCarriedPlanFallsBackToStopped() {
        Roadmap map = gridRoadmap(4, 4);
        int here = seedCell(map, 0, 1);

        assertThat(MapfPlanner.motionEnteringTheCurrentCell(map, null, here))
                .isEqualTo(Roadmap.DIR_NONE);
        assertThat(MapfPlanner.motionEnteringTheCurrentCell(map, new int[] { here }, here))
                .isEqualTo(Roadmap.DIR_NONE);
        assertThat(MapfPlanner.motionEnteringTheCurrentCell(
                map, new int[] { seedCell(map, 0, 0), seedCell(map, 0, 3) }, here))
                .as("a carried plan whose index 1 is not where the vehicle stands says nothing about"
                        + " the edge it came in on")
                .isEqualTo(Roadmap.DIR_NONE);
    }
}
