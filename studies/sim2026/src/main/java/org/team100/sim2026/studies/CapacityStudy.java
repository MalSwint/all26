package org.team100.sim2026.studies;

import org.team100.sim2026.SimRun;
import org.team100.sim2026.scenarios.Scenario;
import org.team100.sim2026.scenarios.FerryVsFerryVariable;

public class CapacityStudy implements Runnable {
    // private static final int CAPACITY = 50;
    private static final int INTAKE_RATE = 25;
    private static final int SHOOT_RATE = 10;

    @Override
    public void run() {
        System.out.println("Red med blue low");
        Scenario scenario = new FerryVsFerryVariable(
                40, INTAKE_RATE, SHOOT_RATE, 10, INTAKE_RATE, SHOOT_RATE);
        SimRun sim = new SimRun(scenario, false);
        sim.run();
        System.out.println("Red high Blue low");
        scenario = new FerryVsFerryVariable(
                80, INTAKE_RATE, SHOOT_RATE, 10, INTAKE_RATE, SHOOT_RATE);
        sim = new SimRun(scenario, false);
        sim.run();
        System.out.println("Red high Blue med");
        scenario = new FerryVsFerryVariable(
                80, INTAKE_RATE, SHOOT_RATE, 40, INTAKE_RATE, SHOOT_RATE);
        sim = new SimRun(scenario, false);
        sim.run();
    }

}
