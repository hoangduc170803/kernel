package org.opentcs.fms.routing.pibt;

/**
 * Deterministic PRNG (mulberry32), ported from the TypeScript engine's
 * {@code makeRng}. PIBT needs randomised tie-breaking for good behaviour; a
 * seeded generator keeps plans reproducible and tests stable.
 *
 * <p>Java {@code int} arithmetic wraps at 32 bits exactly like JS {@code |0} /
 * {@code Math.imul}, and {@code >>>} is the unsigned shift in both languages, so
 * the bit math matches the original 1:1.
 */
public final class Mulberry32 {

  private int a;

  public Mulberry32(int seed) {
    this.a = seed;
  }

  /** Next pseudo-random double in [0, 1). */
  public double next() {
    a = a + 0x6d2b79f5;                       // (a + k) | 0
    int t = (a ^ (a >>> 15)) * (1 | a);       // Math.imul(a ^ (a>>>15), 1|a)
    t = (t + (t ^ (t >>> 7)) * (61 | t)) ^ t; // ... ^ t
    return ((t ^ (t >>> 14)) & 0xFFFFFFFFL) / 4294967296.0; // (... >>> 0) / 2^32
  }
}
