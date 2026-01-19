package org.team100.sim2026;

import org.team100.sim2026.studies.BalancedVsUniformStudy;
import org.team100.sim2026.studies.CapacityStudy;

public class Main {
    public static void main(String... args) {
        System.out.println("simulation starting ...");
        // Runnable study = new BalancedVsUniformStudy();
        Runnable study = new CapacityStudy();
        study.run();
        System.out.println("simulation done.");
    }

}
