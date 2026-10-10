package org.opentcs.fms.kernel.scheduling;

import org.opentcs.data.order.DriveOrder;
import org.opentcs.data.order.TransportOrder;

public final class RetreatHolds {

  public static final String LEG_PROPERTY = "wes:leg";
  public static final String DROPOFF_LEG = "DROPOFF";
  public static final String APPROACH_LEG = "APPROACH";
  private RetreatHolds() {
  }

  public static boolean isDropOffLeg(TransportOrder order) {
    return DROPOFF_LEG.equals(order.getProperty(LEG_PROPERTY));
  }

  public static boolean isApproachLeg(TransportOrder order) {
    return APPROACH_LEG.equals(order.getProperty(LEG_PROPERTY));
  }

  public static boolean isRetreating(TransportOrder order) {
    if (!isDropOffLeg(order)) {
      return false;
    }
    DriveOrder current = order.getCurrentDriveOrder();
    return current != null && current.getDestination() != null
        && DriveOrder.Destination.OP_MOVE.equals(current.getDestination().getOperation());
  }
}
