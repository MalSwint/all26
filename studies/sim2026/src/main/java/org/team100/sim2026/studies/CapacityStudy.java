package org.team100.sim2026.studies;

import org.team100.sim2026.SimRun;
import org.team100.sim2026.scenarios.Scenario;
import org.team100.sim2026.scenarios.UniformVsUniformVariableCapacity;

public class CapacityStudy implements Runnable {
    @Override
    public void run() {
        System.out.println("Red med blue low");
        Scenario scenario = new UniformVsUniformVariableCapacity(40, 10);
        SimRun sim = new SimRun(scenario, false);
        sim.run();
        System.out.println("Red high Blue low");
        scenario = new UniformVsUniformVariableCapacity(80,10);
        sim = new SimRun(scenario, false);
        sim.run();
        System.out.println("Red high Blue med");
        scenario = new UniformVsUniformVariableCapacity(80, 40);
        sim = new SimRun(scenario, false);
        sim.run();

    }

}
