package org.opentcs.fms.kernel;

import java.util.Optional;

import org.opentcs.data.order.TransportOrder;

/**
 * The order view used by one fleet snapshot for both goal and claim decisions.
 *
 * @param order the selected current or pending order
 * @param goal the goal derived from that order, if available
 * @param source where the selected order came from
 */
record VehicleOrderContext(
    TransportOrder order,
    Optional<String> goal,
    Source source) {

  enum Source {
    CURRENT,
    PENDING
  }
}
