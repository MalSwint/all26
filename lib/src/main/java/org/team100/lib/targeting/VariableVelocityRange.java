package org.team100.lib.targeting;

/** Provides a firing solution from elevation and muzzle velocity. */
public class VariableVelocityRange {

    private final Drag d;
    private final double omega;

    public VariableVelocityRange(Drag d, double omega) {
        this.d = d;
        this.omega = omega;
    }

    public FiringSolution get(double v, double elevation) {
        return RangeSolver.getSolution(d, v, omega, elevation);
    }
}
