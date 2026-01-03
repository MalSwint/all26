package org.team100.lib.targeting;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * Provides a firing solution (range and time of flight) by integrating a drag
 * model with initial conditions of elevation and fixed muzzle velocity.
 * 
 * Results are optionally precomputed, and stored in an interpolating map.
 */
public class Range {
    private static final boolean DEBUG = false;
    /** Precomputation lower bound. */
    private static final double MIN_ELEVATION = 0;
    /** Precomputation upper bound. */
    private static final double MAX_ELEVATION = Math.PI / 2;
    /** Precomputation step. */
    private static final double ELEVATION_STEP = 0.01;

    private final Drag m_d;
    private final double m_v;
    private final double m_omega;

    /** key = elevation in radians, value = solution */
    private final InterpolatingTreeMap<Double, FiringSolution> m_map;

    /**
     * @param d        drag model
     * @param v        muzzle speed in m/s
     * @param omega    spin in rad/s, positive is backspin
     * @param useCache precompute results for the full range of elevation
     */
    public Range(Drag d, double v, double omega, boolean useCache) {
        m_d = d;
        m_v = v;
        m_omega = omega;
        if (useCache) {
            m_map = init(d, v, omega);
        } else {
            m_map = null;
        }
    }

    /**
     * @param elevation in radians
     */
    public FiringSolution get(double elevation) {
        if (m_map != null)
            return m_map.get(elevation);
        return RangeSolver.getSolution(m_d, m_v, m_omega, elevation);
    }

    private static InterpolatingTreeMap<Double, FiringSolution> init(Drag d, double v, double omega) {
        InterpolatingTreeMap<Double, FiringSolution> map = new InterpolatingTreeMap<>(
                InverseInterpolator.forDouble(), new FiringSolutionInterpolator());
        if (DEBUG)
            System.out.println("elevation, range, tof");
        for (double elevation = MIN_ELEVATION; elevation <= MAX_ELEVATION; elevation += ELEVATION_STEP) {
            FiringSolution solution = RangeSolver.getSolution(d, v, omega, elevation);
            if (solution == null) {
                // no solution
                continue;
            }
            if (DEBUG)
                System.out.printf("%6.3f, %6.3f, %6.3f\n",
                        elevation, solution.range(), solution.tof());
            map.put(elevation, solution);
        }
        return map;
    }

}
