package org.opentcs.fms.kernel.mapf.adg;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import java.util.Map;

import org.junit.jupiter.api.Test;
import org.opentcs.fms.kernel.mapf.adg.AdgDiagnostics.Reversal;
import org.opentcs.fms.kernel.mapf.adg.AdgDiagnostics.TerminalBlock;

class AdgDiagnosticsTest {

  private static final Map<String, List<String>> ROUTES = Map.of(
      "A", List.of("a0", "a1", "a2"),
      "B", List.of("b0", "b1", "b2"));

  @Test
  void anAcyclicGraphHasNoTrappedNodes() {
    Adg adg = Adg.of(Map.of(new Step("A", 1), List.of(new Step("B", 1))), ROUTES);

    assertThat(AdgDiagnostics.nodesInCycles(adg)).isEmpty();
  }

  @Test
  void aMutualWaitTrapsEveryNodeDownstreamOfBothVehicles() {
    Adg adg = Adg.of(Map.of(
        new Step("A", 1), List.of(new Step("B", 1)),
        new Step("B", 1), List.of(new Step("A", 1))), ROUTES);

    assertThat(AdgDiagnostics.nodesInCycles(adg)).containsExactly(
        new Step("A", 1), new Step("A", 2), new Step("B", 1), new Step("B", 2));
  }

  @Test
  void theCycleCheckCatchesWhatLinkingTheWaitedStepAloneWouldMiss() {
    Adg adg = Adg.of(Map.of(
        new Step("A", 1), List.of(new Step("B", 1)),
        new Step("B", 2), List.of(new Step("A", 1))), ROUTES);

    assertThat(AdgDiagnostics.nodesInCycles(adg)).containsExactly(
        new Step("A", 1), new Step("A", 2), new Step("B", 2));
  }

  @Test
  void aPairThatSwapsDirectionBetweenReplansIsAReversal() {
    Adg before = Adg.of(Map.of(new Step("A", 1), List.of(new Step("B", 1))), ROUTES);
    Adg after = Adg.of(Map.of(new Step("B", 1), List.of(new Step("A", 1))), ROUTES);

    assertThat(AdgDiagnostics.reversals(before, after))
        .containsExactly(new Reversal("A", "B"));
  }

  @Test
  void aPairKeepingItsDirectionIsNotAReversalEvenWhenTheStepsRenumber() {
    Adg before = Adg.of(Map.of(new Step("A", 1), List.of(new Step("B", 1))), ROUTES);
    Adg after = Adg.of(Map.of(new Step("A", 2), List.of(new Step("B", 0))), ROUTES);

    assertThat(AdgDiagnostics.reversals(before, after)).isEmpty();
  }

  @Test
  void aPairConstrainedInBothDirectionsIsNotCountedAsAReversal() {
    Adg before = Adg.of(Map.of(
        new Step("A", 1), List.of(new Step("B", 1)),
        new Step("B", 2), List.of(new Step("A", 0))), ROUTES);
    Adg after = Adg.of(Map.of(new Step("B", 1), List.of(new Step("A", 1))), ROUTES);

    assertThat(AdgDiagnostics.reversals(before, after)).isEmpty();
  }

  @Test
  void aBlockerParkedAtItsLastStepIsReportedAsTerminal() {
    Adg adg = Adg.of(Map.of(new Step("A", 1), List.of(new Step("B", 2))), ROUTES);

    assertThat(AdgDiagnostics.terminalBlocks(adg))
        .containsExactly(new TerminalBlock(new Step("A", 1), new Step("B", 2)));
  }

  @Test
  void aBlockerWithStepsLeftToTakeIsNotTerminal() {
    Adg adg = Adg.of(Map.of(new Step("A", 1), List.of(new Step("B", 1))), ROUTES);

    assertThat(AdgDiagnostics.terminalBlocks(adg)).isEmpty();
  }
}
