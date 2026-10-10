package org.opentcs.fms.kernel.mapf.adg;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

public final class AdgDiagnostics {

  private static final org.slf4j.Logger logger =
      org.slf4j.LoggerFactory.getLogger(AdgDiagnostics.class);

  private static final int SHOWN = 3;

  private AdgDiagnostics() {
  }

  public record Reversal(String waiter, String blocker) {
  }

  public record TerminalBlock(Step waiter, Step blocker) {
  }

  public static void log(Adg before, Adg after) {
    logCycle(after);
    logTerminalBlocks(after);
    logChurn(before, after);
  }

  static void logCycle(Adg adg) {
    List<Step> trapped = nodesInCycles(adg);
    if (trapped.isEmpty()) {
      return;
    }
    logger.error(
        "[MAPF prec-cycle] the dependency graph is CYCLIC: {} nodes cannot be ordered."
            + " Vehicles in the cycle wait for each other until the next replan."
            + " Trapped (vehicle#step, capped): {}",
        trapped.size(), trapped.subList(0, Math.min(12, trapped.size())));
  }

  static void logTerminalBlocks(Adg adg) {
    List<TerminalBlock> terminal = terminalBlocks(adg);
    if (terminal.isEmpty()) {
      return;
    }
    logger.info(
        "[MAPF prec-terminal] {} waits are on a blocker parked at its LAST step, so the"
            + " release condition cannot be met before the next replan: {}",
        terminal.size(), describeTerminal(adg, terminal));
  }

  static void logChurn(Adg before, Adg after) {
    List<Reversal> reversals = reversals(before, after);
    logger.info("[MAPF churn] {} edges over {} vehicle pairs, {} REVERSALS",
        edgeCount(after), pairCount(after), reversals.size());
    for (Reversal reversal : reversals.subList(0, Math.min(SHOWN, reversals.size()))) {
      logger.info(
          "[MAPF prec-flip] {} waited for {}, now {} waits for {} | was {} | now {}",
          reversal.waiter(), reversal.blocker(), reversal.blocker(), reversal.waiter(),
          describeEdges(before, reversal.waiter(), reversal.blocker()),
          describeEdges(after, reversal.blocker(), reversal.waiter()));
    }
  }

  static List<Reversal> reversals(Adg before, Adg after) {
    Map<String, Set<String>> was = waitsFor(before);
    Map<String, Set<String>> now = waitsFor(after);
    List<Reversal> found = new ArrayList<>();
    was.forEach((waiter, blockers) -> {
      for (String blocker : blockers) {
        if (mutual(was, waiter, blocker) || mutual(now, waiter, blocker)) {
          continue;
        }
        if (waits(now, blocker, waiter) && !waits(now, waiter, blocker)) {
          found.add(new Reversal(waiter, blocker));
        }
      }
    });
    found.sort(Comparator.comparing(Reversal::waiter).thenComparing(Reversal::blocker));
    return List.copyOf(found);
  }

  static List<TerminalBlock> terminalBlocks(Adg adg) {
    List<TerminalBlock> terminal = new ArrayList<>();
    adg.dependencies().forEach((waiter, blockers) -> {
      for (Step blocker : blockers) {
        if (blocker.step() >= adg.routeOf(blocker.vehicle()).size() - 1) {
          terminal.add(new TerminalBlock(waiter, blocker));
        }
      }
    });
    terminal.sort(Comparator
        .comparing((TerminalBlock block) -> block.waiter().vehicle())
        .thenComparingInt(block -> block.waiter().step()));
    return List.copyOf(terminal);
  }

  static List<Step> nodesInCycles(Adg adg) {
    Map<Step, List<Step>> successors = new HashMap<>();
    Map<Step, Integer> indegree = new HashMap<>();
    adg.routes().forEach((vehicle, route) -> {
      for (int step = 0; step < route.size(); step++) {
        indegree.putIfAbsent(new Step(vehicle, step), 0);
      }
    });
    adg.routes().forEach((vehicle, route) -> {
      for (int step = 1; step < route.size(); step++) {
        link(successors, indegree, new Step(vehicle, step - 1), new Step(vehicle, step));
      }
    });
    adg.dependencies().forEach((waiter, blockers) -> {
      for (Step blocker : blockers) {
        link(successors, indegree, new Step(blocker.vehicle(), blocker.step() + 1), waiter);
      }
    });

    Deque<Step> ready = new ArrayDeque<>();
    indegree.forEach((node, degree) -> {
      if (degree == 0) {
        ready.add(node);
      }
    });
    int settled = 0;
    while (!ready.isEmpty()) {
      settled++;
      for (Step next : successors.getOrDefault(ready.poll(), List.of())) {
        if (indegree.merge(next, -1, Integer::sum) == 0) {
          ready.add(next);
        }
      }
    }
    if (settled == indegree.size()) {
      return List.of();
    }
    return indegree.entrySet().stream()
        .filter(entry -> entry.getValue() > 0)
        .map(Map.Entry::getKey)
        .sorted(Comparator.comparing(Step::vehicle).thenComparingInt(Step::step))
        .toList();
  }

  private static void link(
      Map<Step, List<Step>> successors, Map<Step, Integer> indegree, Step from, Step to) {
    if (!indegree.containsKey(from) || !indegree.containsKey(to) || from.equals(to)) {
      return;
    }
    successors.computeIfAbsent(from, key -> new ArrayList<>()).add(to);
    indegree.merge(to, 1, Integer::sum);
  }

  private static Map<String, Set<String>> waitsFor(Adg adg) {
    Map<String, Set<String>> byVehicle = new TreeMap<>();
    adg.dependencies().forEach((waiter, blockers) -> {
      Set<String> others = byVehicle.computeIfAbsent(waiter.vehicle(), key -> new TreeSet<>());
      for (Step blocker : blockers) {
        others.add(blocker.vehicle());
      }
    });
    return byVehicle;
  }

  private static boolean waits(Map<String, Set<String>> graph, String waiter, String blocker) {
    return graph.getOrDefault(waiter, Set.of()).contains(blocker);
  }

  private static boolean mutual(Map<String, Set<String>> graph, String one, String other) {
    return waits(graph, one, other) && waits(graph, other, one);
  }

  private static int edgeCount(Adg adg) {
    return adg.dependencies().values().stream().mapToInt(List::size).sum();
  }

  private static int pairCount(Adg adg) {
    Set<String> pairs = new LinkedHashSet<>();
    adg.dependencies().forEach((waiter, blockers) -> {
      for (Step blocker : blockers) {
        pairs.add(waiter.vehicle().compareTo(blocker.vehicle()) < 0
            ? waiter.vehicle() + "|" + blocker.vehicle()
            : blocker.vehicle() + "|" + waiter.vehicle());
      }
    });
    return pairs.size();
  }

  private static String describeEdges(Adg adg, String waiter, String blocker) {
    List<String> rendered = new ArrayList<>();
    adg.dependencies().forEach((step, blockers) -> {
      if (!step.vehicle().equals(waiter)) {
        return;
      }
      for (Step candidate : blockers) {
        if (candidate.vehicle().equals(blocker)) {
          rendered.add(step + "@" + adg.pointOf(step)
              + "<-" + candidate + "@" + adg.pointOf(candidate));
        }
      }
    });
    return rendered.isEmpty() ? "none" : String.join(" ", rendered);
  }

  private static String describeTerminal(Adg adg, List<TerminalBlock> terminal) {
    List<String> rendered = new ArrayList<>();
    for (TerminalBlock block : terminal.subList(0, Math.min(SHOWN, terminal.size()))) {
      rendered.add(block.waiter() + "@" + adg.pointOf(block.waiter())
          + "<-" + block.blocker() + "@" + adg.pointOf(block.blocker()));
    }
    return String.join(" ", rendered);
  }
}
