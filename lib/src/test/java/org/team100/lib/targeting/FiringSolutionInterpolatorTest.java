package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class FiringSolutionInterpolatorTest {
    private static final double DELTA = 0.001;

    @Test
    void test0() {
        FiringSolutionInterpolator fsi = new FiringSolutionInterpolator();
        FiringSolution a = new FiringSolution(1, 2, 1);
        FiringSolution b = new FiringSolution(5, 6, 1);
        FiringSolution i = fsi.interpolate(a, b, 0.5);
        assertEquals(3, i.range(), DELTA);
        assertEquals(4, i.tof(), DELTA);
    }
}
