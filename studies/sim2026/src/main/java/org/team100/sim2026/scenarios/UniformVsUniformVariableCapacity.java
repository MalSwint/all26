package org.team100.sim2026.scenarios;

import org.team100.sim2026.Alliance;
import org.team100.sim2026.AllianceColor;
import org.team100.sim2026.SimRun;

public class UniformVsUniformVariableCapacity implements Scenario {
    private final int redCapacity;
    private final int blueCapacity;

    public UniformVsUniformVariableCapacity(int redCapacity, int blueCapacity) {
        this.redCapacity = redCapacity;
        this.blueCapacity = blueCapacity;
    }

    @Override
    public Alliance red(SimRun sim) {
        return Alliance.uniform("uniform", redCapacity, AllianceColor.RED, sim);
    }

    @Override
    public Alliance blue(SimRun sim) {
        return Alliance.uniform("uniform", blueCapacity, AllianceColor.BLUE, sim);
    }
}
