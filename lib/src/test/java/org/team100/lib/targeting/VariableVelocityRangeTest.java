package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class VariableVelocityRangeTest {
    private static final double DELTA = 0.001;

    @Test
    void testRange() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        VariableVelocityRange r = new VariableVelocityRange(d, 50, false);
        FiringSolution s = r.get(8, Math.PI / 4);
        assertEquals(2.825, s.range(), DELTA);
        assertEquals(1.011, s.tof(), DELTA);
    }

    @Test
    void testRangeCached() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        VariableVelocityRange r = new VariableVelocityRange(d, 50, true);
        FiringSolution s = r.get(8, Math.PI / 4);
        assertEquals(2.825, s.range(), 0.01);
        assertEquals(1.011, s.tof(), DELTA);
    }
}
