package org.team100.lib.trajectory.timing;

import org.team100.lib.trajectory.path.PathSE2Entry;
import org.team100.lib.trajectory.path.PathSE2Point;
import org.team100.lib.trajectory.path.PathUtil;

public class TimingUtil {
    private static final boolean DEBUG = false;

    /**
     * Linear interpolation by time.
     * 
     * Velocity of this state is the initial velocity.
     * Acceleration of this state is constant through the whole arc.
     */
    public static TimedStateSE2 interpolate(TimedStateSE2 a, TimedStateSE2 b, double delta_t) {
        if (delta_t < 0)
            throw new IllegalArgumentException("delta_t must be non-negative");
        if (DEBUG)
            System.out.println("lerp");
        double tLerp = a.m_timeS + delta_t;
        double vLerp = a.m_velocityM_S + a.m_accelM_S_S * delta_t;
        double pathwiseDistance = a.m_velocityM_S * delta_t + 0.5 * a.m_accelM_S_S * delta_t * delta_t;

        PathSE2Point aPoint = a.m_point;
        PathSE2Point bPoint = b.m_point;
        double distanceBetween = aPoint.distanceCartesian(bPoint);
        double interpolant = pathwiseDistance / distanceBetween;
        if (Double.isNaN(interpolant)) {
            interpolant = 1.0;
        }

        // TODO: pass t interpolant, not just spatial one
        double timeInterp = delta_t / (b.m_timeS - a.m_timeS);
        if (DEBUG)
            System.out.printf("t0 %f t1 %f delta t %f timeInterp %f\n",
                    a.m_timeS, b.m_timeS, delta_t, timeInterp);

        if (DEBUG)
            System.out.printf("tlerp %f\n", tLerp);
        PathSE2Entry aEntry = new PathSE2Entry(a.m_parameter, aPoint);
        PathSE2Entry bEntry = new PathSE2Entry(b.m_parameter, bPoint);

        PathSE2Entry entryLerp = PathUtil.interpolate(aEntry, bEntry, timeInterp);
        
        return new TimedStateSE2(
                entryLerp.parameter(),
                entryLerp.point(),
                tLerp,
                vLerp,
                a.m_accelM_S_S);
    }

}
