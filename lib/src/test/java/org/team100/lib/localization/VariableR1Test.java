package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class VariableR1Test {
    private static final double DELTA = 0.001;

    @Test
    void testAdd0() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(1, 1);
        VariableR1 c = VariableR1.add(a, b);
        assertEquals(1, c.mean(), DELTA);
        assertEquals(2, c.variance(), DELTA);
    }

    // MIXTURE MODEL
    //
    // Never becomes more confident than the measurement

    @Test
    void testFuse1a() {
        // Fuse with self
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(0, 1);
        VariableR1 c = VariableR1.fuse1(a, b);
        assertEquals(0, c.mean(), DELTA);
        // Self-fusion has no effect: this isn't like "more evidence".
        assertEquals(1, c.variance(), DELTA);
    }

    @Test
    void testFuse1b() {
        VariableR1 a = new VariableR1(0, 1);
        // "don't know" variance
        VariableR1 b = new VariableR1(1, 100);
        VariableR1 c = VariableR1.fuse1(a, b);
        assertEquals(0, c.mean(), DELTA);
        // Overresponds to the don't-know variance
        assertEquals(1.98, c.variance(), DELTA);
    }

    @Test
    void testFuse1c() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(1, 1);
        VariableR1 c = VariableR1.fuse1(a, b);
        // Equal variance: mean is in the middle.
        assertEquals(0.5, c.mean(), DELTA);
        // Dispersion of the mean adds 0.25
        assertEquals(1.25, c.variance(), DELTA);
    }

    // INVERSE-VARIANCE WEIGHTING
    //
    // Becomes overconfident, ignores mean dispersion.

    @Test
    void testFuse2a() {
        // Fuse with self
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(0, 1);
        VariableR1 c = VariableR1.fuse2(a, b);
        assertEquals(0, c.mean(), DELTA);
        // Self-fusion increases confidence (too much)
        assertEquals(0.5, c.variance(), DELTA);
    }

    @Test
    void testFuse2b() {
        VariableR1 a = new VariableR1(0, 1);
        // "don't know" variance
        VariableR1 b = new VariableR1(1, 100);
        VariableR1 c = VariableR1.fuse2(a, b);
        // Mostly ignores the uncertain input
        assertEquals(0.01, c.mean(), DELTA);
        // Note the variance here is about the same
        // but *less* than the previous, which is wrong.
        assertEquals(0.99, c.variance(), DELTA);
    }

    @Test
    void testFuse2c() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(1, 1);
        VariableR1 c = VariableR1.fuse2(a, b);
        assertEquals(0.5, c.mean(), DELTA);
        // variance ignores mean dispersion
        assertEquals(0.5, c.variance(), DELTA);
    }

    // COVARIANCE INFLATION
    //
    // Inverse-variance weighting with mean dispersion

    @Test
    void testFuse3a() {
        // Self fusion
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(0, 1);
        VariableR1 c = VariableR1.fuse3(a, b);
        assertEquals(0, c.mean(), DELTA);
        // Increase confidence too much if means are the same
        assertEquals(0.5, c.variance(), DELTA);
    }

    @Test
    void testFuse3b() {
        VariableR1 a = new VariableR1(0, 1);
        // don't know
        VariableR1 b = new VariableR1(1, 100);
        VariableR1 c = VariableR1.fuse3(a, b);
        assertEquals(0.01, c.mean(), DELTA);
        // Overresponds to mean dispersion
        assertEquals(1.99, c.variance(), DELTA);
    }

    @Test
    void testFuse3c() {
        // Means are different by 1
        // Stddev is 1
        // A good result would be
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(1, 1);
        VariableR1 c = VariableR1.fuse3(a, b);
        // Equal variance -> expectation in the middle
        assertEquals(0.5, c.mean(), DELTA);
        // Respects mean dispersion
        assertEquals(1.25, c.variance(), DELTA);
    }

    // BAYESIAN
    //
    // Identical to inverse-variance weighting.

    @Test
    void testFuse4a() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(0, 1);
        VariableR1 c = VariableR1.fuse4(a, b);
        assertEquals(0, c.mean(), DELTA);
        assertEquals(0.5, c.variance(), DELTA);
    }

    @Test
    void testFuse4b() {
        VariableR1 a = new VariableR1(0, 1);
        // don't know
        VariableR1 b = new VariableR1(1, 100);
        VariableR1 c = VariableR1.fuse4(a, b);
        assertEquals(0.01, c.mean(), DELTA);
        assertEquals(0.99, c.variance(), DELTA);
    }

    @Test
    void testFuse4c() {
        VariableR1 a = new VariableR1(0, 1);
        VariableR1 b = new VariableR1(1, 1);
        VariableR1 c = VariableR1.fuse4(a, b);
        assertEquals(0.5, c.mean(), DELTA);
        // variance ignores mean dispersion
        assertEquals(0.5, c.variance(), DELTA);
    }

    @Test
    void testWeight0() {
        // Same variance.
        double varA = 1;
        double varB = 1;
        double w = VariableR1.weight(varA, varB);
        // Equal weights.
        assertEquals(0.5, w, DELTA);
    }

}
