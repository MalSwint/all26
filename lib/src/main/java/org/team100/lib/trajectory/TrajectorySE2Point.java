package org.team100.lib.trajectory;

import org.team100.lib.trajectory.path.PathSE2Point;
import org.team100.lib.util.Math100;

public class TrajectorySE2Point {
    private static final boolean DEBUG = false;

    /**
     * Point on a path, with heading rate and curvature.
     */
    private final PathSE2Point m_point;
    /**
     * Time we achieve this state.
     */
    private final double m_timeS;
    /**
     * Instantaneous pathwise velocity, m/s.
     */
    private final double m_velocityM_S;
    /**
     * Pathwise acceleration for the timespan after this state, m/s^2. It's computed
     * by looking at the velocity of the next state, and the distance to get there.
     */
    private final double m_accelM_S_S;

    public TrajectorySE2Point(
            PathSE2Point point,
            double t,
            double velocity,
            double acceleration) {
        m_point = point;
        m_timeS = t;
        m_velocityM_S = velocity;
        m_accelM_S_S = acceleration;
    }

    /** path point */
    public PathSE2Point point() {
        return m_point;
    }

    /** Instant this point is reached, seconds */
    public double time() {
        return m_timeS;
    }

    /** Instantaneous pathwise velocity, m/s */
    public double velocity() {
        return m_velocityM_S;
    }

    /** Instantaneous pathwise (not centripetal) acceleration, m/s^2 */
    public double accel() {
        return m_accelM_S_S;
    }

    @Override
    public String toString() {
        return String.format("TrajectorySE2Point [ %s,  %5.3f, %5.3f, %5.3f ]",
                point(), time(), velocity(), accel());
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof TrajectorySE2Point)) {
            if (DEBUG)
                System.out.println("wrong type");
            return false;
        }
        TrajectorySE2Point ts = (TrajectorySE2Point) other;
        if (!point().equals(ts.point())) {
            if (DEBUG)
                System.out.println("wrong state");
            return false;
        }
        if (!Math100.epsilonEquals(time(), ts.time())) {
            if (DEBUG)
                System.out.println("wrong time");
            return false;
        }
        return true;
    }

}
