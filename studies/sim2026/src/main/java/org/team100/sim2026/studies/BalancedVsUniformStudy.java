package org.team100.sim2026.studies;

import org.team100.sim2026.SimRun;
import org.team100.sim2026.scenarios.BalancedVsBalanced;
import org.team100.sim2026.scenarios.BalancedVsUniform;
import org.team100.sim2026.scenarios.Scenario;
import org.team100.sim2026.scenarios.UniformVsUniform;

public class BalancedVsUniformStudy implements Runnable {
    @Override
    public void run() {
        System.out.println("Balanced vs Uniform");
        Scenario scenario = new BalancedVsUniform();
        SimRun sim = new SimRun(scenario, false);
        sim.run();
        System.out.println("Balanced vs Balanced");
        scenario = new BalancedVsBalanced();
        sim = new SimRun(scenario, false);
        sim.run();
        System.out.println("Uniform vs Uniform");
        scenario = new UniformVsUniform();
        sim = new SimRun(scenario, false);
        sim.run();
    }
}
