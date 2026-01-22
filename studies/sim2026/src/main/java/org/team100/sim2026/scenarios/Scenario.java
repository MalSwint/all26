package org.team100.sim2026.scenarios;

import org.team100.sim2026.Alliance;
import org.team100.sim2026.SimRun;

/** Factory for alliances for a single simulation run. */
public interface Scenario {
    Alliance red(SimRun sim);

    Alliance blue(SimRun sim);
}
