package org.team100.sim2026.scenarios;

import org.team100.sim2026.Alliance;
import org.team100.sim2026.AllianceColor;
import org.team100.sim2026.SimRun;

public class BalancedVsBalanced implements Scenario {
    @Override
    public Alliance red(SimRun sim) {
        return Alliance.balanced("balanced", AllianceColor.RED, sim);
    }

    @Override
    public Alliance blue(SimRun sim) {
        return Alliance.balanced("balanced", AllianceColor.BLUE, sim);
    }
}
