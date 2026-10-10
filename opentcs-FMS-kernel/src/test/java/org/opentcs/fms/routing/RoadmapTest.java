package org.opentcs.fms.routing;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.junit.jupiter.api.Test;

class RoadmapTest {

  @Test
  void mapsNamesToDenseIds() {
    Roadmap r = GridFixture.grid(2, 2);
    assertThat(r.size()).isEqualTo(4);
    assertThat(r.has("r0c0")).isTrue();
    assertThat(r.idOf("r0c0")).isNotNull();
    assertThat(r.nameOf(r.idOf("r0c0"))).isEqualTo("r0c0");
    assertThat(r.idOf("nope")).isNull();
  }

  @Test
  void candidatesIncludeSelfLast() {
    Roadmap r = GridFixture.grid(3, 3);
    int center = GridFixture.id(r, 1, 1);
    int[] cand = r.candidates(center);
    assertThat(cand).hasSize(5); // 4 neighbours + self
    assertThat(cand[cand.length - 1]).isEqualTo(center);
  }

  @Test
  void bidirectionalEdgesGoBothWays() {
    Roadmap r = GridFixture.grid(2, 1);
    int a = GridFixture.id(r, 0, 0);
    int b = GridFixture.id(r, 0, 1);
    assertThat(r.distance(a, b)).isEqualTo(1);
    assertThat(r.distance(b, a)).isEqualTo(1);
  }

  @Test
  void directedEdgeIsForwardOnly() {
    Roadmap r = new Roadmap(new Roadmap.Model(
        List.of(new Roadmap.Point("a", 0, 0, 0), new Roadmap.Point("b", 1, 0, 0)),
        List.of(new Roadmap.Path("a->b", "a", "b", 1, false))));
    int a = r.idOf("a");
    int b = r.idOf("b");
    assertThat(r.distance(a, b)).isEqualTo(1);
    assertThat(r.distance(b, a)).isGreaterThanOrEqualTo(Roadmap.INF);
  }

  @Test
  void derivesDropOffDepthAlongDeclaredLaneAxis() {
    Roadmap r = new Roadmap(new Roadmap.Model(
        List.of(
            new Roadmap.Point("door", 0, 0, 0),
            new Roadmap.Point("slot1", 0, 1, 0),
            new Roadmap.Point("slot2", 0, 2, 0),
            new Roadmap.Point("slot3", 0, 3, 0),
            new Roadmap.Point("plain", 4, 0, 0),
            new Roadmap.Point("wrongDoor", 5, 0, 0),
            new Roadmap.Point("wrongSlot", 6, 0, 0)),
        List.of(
            new Roadmap.Path("door-slot1", "door", "slot1", 1, false),
            new Roadmap.Path("slot1-slot2", "slot1", "slot2", 1, false),
            new Roadmap.Path("slot2-slot3", "slot2", "slot3", 1, false),
            new Roadmap.Path("wrongDoor-wrongSlot", "wrongDoor", "wrongSlot", 1, false)),
        Set.of(), Set.of("slot1", "slot2", "slot3", "wrongSlot"), true));

    assertThat(r.dropOffDepth(r.idOf("slot1"))).isEqualTo(1);
    assertThat(r.dropOffDepth(r.idOf("slot2"))).isEqualTo(2);
    assertThat(r.dropOffDepth(r.idOf("slot3"))).isEqualTo(3);
    assertThat(r.dropOffDepth(r.idOf("plain"))).isZero();
    assertThat(r.dropOffDepth(r.idOf("wrongSlot"))).isZero();
  }

  @Test
  void cargoSlotPointsCostMoreToEnter() {
    Roadmap r = detourMap(Set.of("mid"));
    assertThat(r.weightOf(r.idOf("start"))).isEqualTo(1);
    assertThat(r.weightOf(r.idOf("mid"))).isGreaterThan(r.weightOf(r.idOf("start")));
    assertThat(r.maxVertexWeight()).isEqualTo(r.weightOf(r.idOf("mid")));
  }

  @Test
  void hopShortestRouteLosesToTheCheaperDetour() {
    Roadmap plain = detourMap(Set.of());
    assertThat(plain.distance(plain.idOf("start"), plain.idOf("goal"))).isEqualTo(2);

    Roadmap surcharged = detourMap(Set.of("mid"));
    int viaMid = surcharged.weightOf(surcharged.idOf("mid")) + 1;
    int viaDetour = 3;
    assertThat(surcharged.distance(surcharged.idOf("start"), surcharged.idOf("goal")))
        .isEqualTo(Math.min(viaMid, viaDetour));
  }

  @Test
  void unevenWeightsSurviveRepeatedRelaxation() {
    List<Roadmap.Point> points = new ArrayList<>();
    List<Roadmap.Path> paths = new ArrayList<>();
    Set<String> costly = new HashSet<>();
    int n = 40;
    for (int i = 0; i < n; i++) {
      points.add(new Roadmap.Point("p" + i, i, 0, 0));
      if (i % 2 == 1) {
        costly.add("p" + i);
      }
    }
    for (int i = 0; i + 1 < n; i++) {
      paths.add(new Roadmap.Path("e" + i, "p" + i, "p" + (i + 1), 1, true));
      if (i + 2 < n) {
        paths.add(new Roadmap.Path("skip" + i, "p" + i, "p" + (i + 2), 1, true));
      }
    }
    Roadmap r = new Roadmap(new Roadmap.Model(points, paths, costly));

    int skipsAcrossEvenPoints = (n - 2) / 2;
    assertThat(r.distance(r.idOf("p0"), r.idOf("p" + (n - 1))))
        .isEqualTo(skipsAcrossEvenPoints + r.weightOf(r.idOf("p" + (n - 1))));
  }

  private static Roadmap detourMap(Set<String> cargoSlotPoints) {
    List<Roadmap.Point> points = List.of(
        new Roadmap.Point("start", 0, 0, 0),
        new Roadmap.Point("mid", 1, 0, 0),
        new Roadmap.Point("goal", 2, 0, 0),
        new Roadmap.Point("d1", 0, 1, 0),
        new Roadmap.Point("d2", 2, 1, 0));
    List<Roadmap.Path> paths = List.of(
        new Roadmap.Path("start-mid", "start", "mid", 1, true),
        new Roadmap.Path("mid-goal", "mid", "goal", 1, true),
        new Roadmap.Path("start-d1", "start", "d1", 1, true),
        new Roadmap.Path("d1-d2", "d1", "d2", 1, true),
        new Roadmap.Path("d2-goal", "d2", "goal", 1, true));
    return new Roadmap(new Roadmap.Model(points, paths, cargoSlotPoints));
  }

  @Test
  void unreachableIsInfiniteAndSelfIsZero() {
    Roadmap r = new Roadmap(new Roadmap.Model(
        List.of(new Roadmap.Point("a", 0, 0, 0), new Roadmap.Point("b", 5, 5, 0)),
        List.of()));
    int a = r.idOf("a");
    int b = r.idOf("b");
    assertThat(r.distance(a, b)).isGreaterThanOrEqualTo(Roadmap.INF);
    assertThat(r.distance(a, a)).isZero();
  }

  // --- kinematics: what a rolling vehicle is allowed to do next ----------------

  @Test
  void aStoppedAgentKeepsEveryGeometricOption() {
    Roadmap r = GridFixture.grid(3, 3);
    int center = GridFixture.id(r, 1, 1);

    assertThat(r.kinematicCandidates(center, Roadmap.DIR_NONE))
        .containsExactly(r.candidates(center));
  }

  @Test
  void aMovingAgentMayOnlyContinueOrBrake() {
    Roadmap r = GridFixture.grid(3, 3);
    int center = GridFixture.id(r, 1, 1);
    int east = GridFixture.id(r, 1, 2);

    int[] options = r.kinematicCandidates(center, Roadmap.DIR_POS_X);

    assertThat(options).containsExactly(east, center);
    assertThat(options[options.length - 1]).isEqualTo(center);
  }

  @Test
  void aMovingAgentCannotTurnOrReverseWithoutBraking() {
    Roadmap r = GridFixture.grid(3, 3);
    int center = GridFixture.id(r, 1, 1);

    assertThat(r.kinematicCandidates(center, Roadmap.DIR_POS_X))
        .doesNotContain(GridFixture.id(r, 1, 0))
        .doesNotContain(GridFixture.id(r, 0, 1));
  }

  @Test
  void brakingTakesOneTickAndThenEveryOptionIsBackAgain() {
    Roadmap r = GridFixture.grid(3, 3);
    int center = GridFixture.id(r, 1, 1);

    int afterBraking = r.motionAfter(center, center);

    assertThat(afterBraking).isEqualTo(Roadmap.DIR_NONE);
    assertThat(r.kinematicCandidates(center, afterBraking)).contains(GridFixture.id(r, 1, 0));
  }

  @Test
  void motionFollowsTravelDirectionNotTheNose() {
    Roadmap r = new Roadmap(new Roadmap.Model(
        List.of(new Roadmap.Point("a", 0, 0, 0), new Roadmap.Point("b", 1, 0, 0)),
        List.of(new Roadmap.Path("a->b", "a", "b", 1, true, true))));
    int a = r.idOf("a");
    int b = r.idOf("b");

    assertThat(r.requiredNose(a, b))
        .as("a ~rev edge is driven backwards, so the nose points away from the travel")
        .isEqualTo(Roadmap.DIR_NEG_X);
    assertThat(r.motionAfter(a, b)).isEqualTo(Roadmap.DIR_POS_X);
    assertThat(r.kinematicCandidates(a, Roadmap.DIR_POS_X)).containsExactly(b, a);
  }

  @Test
  void callersMayShuffleTheReturnedArrayInPlace() {
    Roadmap r = GridFixture.grid(3, 3);
    int center = GridFixture.id(r, 1, 1);

    int[] first = r.kinematicCandidates(center, Roadmap.DIR_POS_X);
    first[0] = -99;

    assertThat(r.kinematicCandidates(center, Roadmap.DIR_POS_X)).doesNotContain(-99);
  }
}
