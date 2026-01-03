package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.RepeatedInterpolator;

public class FiringSolutionBiInterpolatorTest {
    private static final double DELTA = 0.001;

    @Test
    void test0() {
        RepeatedInterpolator<FiringSolution> fsbi = new RepeatedInterpolator<>(
                new FiringSolutionInterpolator());
        FiringSolution v11 = new FiringSolution(1, 1, 1);
        FiringSolution v12 = new FiringSolution(1, 2, 1);
        FiringSolution v21 = new FiringSolution(2, 1, 1);
        FiringSolution v22 = new FiringSolution(2, 2, 1);
        FiringSolution i00 = fsbi.interpolate(v11, v12, v21, v22, 0, 0);
        assertEquals(1, i00.range(), DELTA);
        assertEquals(1, i00.tof(), DELTA);
        FiringSolution i01 = fsbi.interpolate(v11, v12, v21, v22, 0, 1);
        assertEquals(1, i01.range(), DELTA);
        assertEquals(2, i01.tof(), DELTA);
        FiringSolution i5 = fsbi.interpolate(v11, v12, v21, v22, 0.5, 0.5);
        assertEquals(1.5, i5.range(), DELTA);
        assertEquals(1.5, i5.tof(), DELTA);

    }

}
