package org.opentcs.fms.kernel;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Pose;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.DriveOrder;
import org.opentcs.data.order.Route;
import org.opentcs.data.order.TransportOrder;
import org.opentcs.drivers.vehicle.LoadHandlingDevice;
import org.opentcs.fms.kernel.mapf.BodyState;
import org.opentcs.fms.routing.Roadmap;

class BodyStatesTest {

    private static final Map<String, String> AT_0008 = Map.of("V01", "0008");

    @Test
    void aDeckReportedFullMakesTheVehicleLoaded() {
        BodyState state = BodyStates.of(
                List.of(withDeck(true)), AT_0008, vehicle -> Optional.empty());

        assertThat(state.loaded("V01"))
                .describedAs("VDA5050 state.loads[] reaches the kernel as a full LoadHandlingDevice,"
                        + " and that is now the only source of the loaded flag")
                .isTrue();
    }

    @Test
    void aPalletCarriedOnAMoveLegCountsAsLoaded() {
        BodyState state = BodyStates.of(
                List.of(withDeck(true)), AT_0008,
                vehicle -> Optional.of(order("3033", DriveOrder.Destination.OP_MOVE)));

        assertThat(state.loaded("V01"))
                .describedAs("the APPROACH leg of a delivery ends in MOVE while the pallet is on the"
                        + " deck; reading the leg operation reported this vehicle as empty and let the"
                        + " footprint shrink to the empty swept radius")
                .isTrue();
    }

    @Test
    void anEmptyDeckIsNotLoadedEvenWhenTheLegEndsInLiftDown() {
        BodyState state = BodyStates.of(
                List.of(withDeck(false)), AT_0008, vehicle -> Optional.of(order("3033", "liftDown")));

        assertThat(state.loaded("V01"))
                .describedAs("the drive order operation no longer influences the flag at all")
                .isFalse();
    }

    @Test
    void aDeviceReportedNotFullDoesNotCount() {
        BodyState state = BodyStates.of(
                List.of(new Vehicle("V01")
                        .withLoadHandlingDevices(List.of(new LoadHandlingDevice("LHD-0", false)))),
                AT_0008, vehicle -> Optional.empty());

        assertThat(state.loaded("V01")).isFalse();
    }

    @Test
    void aVehicleWithoutAReportedPositionIsSkipped() {
        BodyState state = BodyStates.of(
                List.of(withDeck(true)), Map.of(), vehicle -> Optional.empty());

        assertThat(state.loaded("V01")).isFalse();
        assertThat(state.axisByVehicle()).isEmpty();
    }

    @Test
    void theNoseComesFromTheReportedPoseWhenTheVehicleStandsWhereItsPlanStarts() {
        BodyState state = BodyStates.of(
                List.of(withDeck(true)
                        .withCurrentPosition(new Point("0008").getReference())
                        .withPose(new Pose(new Triple(42000, 0, 0), 90))),
                AT_0008, vehicle -> Optional.empty());

        assertThat(state.axisOf("V01")).isEqualTo(Roadmap.DIR_POS_Y);
    }

    @Test
    void aPoseMeasuredCellsBackNeverDescribesTheCellThePlanStartsFrom() {
        Vehicle reversingIntoAJunction = withDeck(true)
                .withCurrentPosition(new Point("0323").getReference())
                .withPose(new Pose(new Triple(26350, -1350, 0), 0));
        TransportOrder runningDownTheColumn = driving(
                at("0006", 25400, -2300), at("0001", 25400, -3050), at("0505", 25400, -3800));

        BodyState state = BodyStates.of(
                List.of(reversingIntoAJunction), Map.of("V01", "0001"),
                vehicle -> Optional.of(runningDownTheColumn));

        assertThat(state.axisOf("V01"))
                .describedAs("the vehicle sits three cells back with its nose east so it can reverse"
                        + " westwards; seeding that nose at 0001 invented a quarter turn there, and the"
                        + " swept radius of that phantom turn refused the straight run down to 0505")
                .isEqualTo(Roadmap.DIR_NEG_Y);
    }

    @Test
    void backingIntoACellPointsTheNoseAgainstTheDirectionOfTravel() {
        BodyState state = BodyStates.of(
                List.of(withDeck(true).withCurrentPosition(new Point("0521").getReference())),
                Map.of("V01", "0516"),
                vehicle -> Optional.of(reversing(at("0521", 24450, -6050), at("0516", 24450, -5300))));

        assertThat(state.axisOf("V01"))
                .describedAs("a vehicle backing up the lane travels +Y with its nose at -Y, and reading"
                        + " the direction of travel as the nose put every such body across the lane")
                .isEqualTo(Roadmap.DIR_NEG_Y);
    }

    private static Vehicle withDeck(boolean full) {
        return new Vehicle("V01")
                .withLoadHandlingDevices(full ? List.of(new LoadHandlingDevice("LHD-0", true)) : List.of());
    }

    private static Point at(String name, long x, long y) {
        return new Point(name).withPose(new Pose(new Triple(x, y, 0), 0.0));
    }

    private static TransportOrder driving(Point... points) {
        return routed(Vehicle.Orientation.FORWARD, points);
    }

    private static TransportOrder reversing(Point... points) {
        return routed(Vehicle.Orientation.BACKWARD, points);
    }

    private static TransportOrder routed(Vehicle.Orientation orientation, Point... points) {
        List<Route.Step> steps = new ArrayList<>();
        for (int i = 1; i < points.length; i++) {
            steps.add(new Route.Step(null, points[i - 1], points[i], orientation, i - 1, 1));
        }
        Point last = points[points.length - 1];
        return new TransportOrder(
                "APPROACH-V01-" + last.getName(),
                List.of(new DriveOrder(
                        "leg-" + last.getName(),
                        new DriveOrder.Destination(last.getReference())
                                .withOperation(DriveOrder.Destination.OP_MOVE))
                        .withRoute(new Route(steps))))
                .withCurrentDriveOrderIndex(0);
    }

    private static TransportOrder order(String destination, String operation) {
        return new TransportOrder(
                "APPROACH-V01-" + destination,
                List.of(new DriveOrder(
                        "leg-" + destination,
                        new DriveOrder.Destination(new Point(destination).getReference())
                                .withOperation(operation))));
    }
}
