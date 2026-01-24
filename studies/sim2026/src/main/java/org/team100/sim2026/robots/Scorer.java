package org.team100.sim2026.robots;

import org.team100.sim2026.AllianceColor;
import org.team100.sim2026.SimRun;

/**
 * Active: Intake and score in our zone.
 * Inctive: Intake in our zone until full.
 */
public class Scorer extends Robot {

    public Scorer(AllianceColor alliance, String name, int capacity,
            int intakeRate, int shootRate, int initialCount, SimRun sim) {
        super(alliance, name, capacity,
                intakeRate, shootRate, initialCount, sim);
    }

    @Override
    void auton() {
        intakeAndScore();
    }

    @Override
    void active() {
        intakeAndScore();
    }

    @Override
    void inactive() {
        intakeOnly();
    }

}
