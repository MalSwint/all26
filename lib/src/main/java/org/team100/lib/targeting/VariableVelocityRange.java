package org.team100.lib.targeting;

import org.team100.lib.util.NestedInterpolatingTreeMap;

import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * Provides a firing solution from elevation and muzzle velocity.
 * Uses a constant spin rate, which is probably wrong. Maybe spin rate is a
 * function of velocity?
 */
public class VariableVelocityRange {
    /**
     * Precomputation lower bound. Very low velocities don't work well with the
     * solvers and they're not useful anyway.
     */
    private static final double MIN_V = 3;
    /**
     * Precomputation upper bound. Very high velocity is possible but maybe not
     * useful.
     */
    private static final double MAX_V = 20;
    /** Precomputation step. */
    private static final double V_STEP = 0.5;
    /** Precomputation lower bound. */
    private static final double MIN_ELEVATION = 0;
    /** Precomputation upper bound. */
    private static final double MAX_ELEVATION = Math.PI / 2;
    /** Precomputation step. */
    private static final double ELEVATION_STEP = 0.05;

    private final Drag d;
    private final double omega;
    /**
     * Cache.
     * 
     * It won't take very much space (tens of KB), but computing all the values
     * requires running two layers of iterative solvers many times, so it's slow
     * to create.
     * 
     * The step values above came from fiddling with testJacobian().
     * 
     * 
     * key1 = velocity (m/s)
     * key2 = elevation (rad)
     * value = solution
     */
    private final NestedInterpolatingTreeMap<Double, FiringSolution> m_map;

    public VariableVelocityRange(Drag d, double omega, boolean useCache) {
        this.d = d;
        this.omega = omega;
        if (useCache) {
            m_map = init(d, omega);
        } else {
            m_map = null;
        }
    }

    public FiringSolution get(double v, double elevation) {
        if (m_map != null)
            return m_map.get(v, elevation);
        return RangeSolver.getSolution(d, v, omega, elevation);
    }

    /**
     * Pre-populate the map.
     */
    private static NestedInterpolatingTreeMap<Double, FiringSolution> init(Drag d, double omega) {
        NestedInterpolatingTreeMap<Double, FiringSolution> map = new NestedInterpolatingTreeMap<>(
                InverseInterpolator.forDouble(), new FiringSolutionInterpolator());
        for (double v = MIN_V; v < MAX_V; v += V_STEP) {
            for (double elevation = MIN_ELEVATION; elevation < MAX_ELEVATION; elevation += ELEVATION_STEP) {
                FiringSolution solution = RangeSolver.getSolution(d, v, omega, elevation);
                if (solution == null) {
                    // no solution
                    continue;
                }
                map.put(v, elevation, solution);
            }
        }
        return map;
    }
}
