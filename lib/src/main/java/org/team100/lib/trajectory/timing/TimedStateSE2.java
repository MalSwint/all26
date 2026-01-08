package org.team100.lib.trajectory.timing;

import org.team100.lib.trajectory.path.PathSE2Point;
import org.team100.lib.trajectory.path.PathSE2Parameter;
import org.team100.lib.util.Math100;

/**
 * Represents a state within a trajectory in SE(2).
 */
public class TimedStateSE2 {
    private static final boolean DEBUG = false;

    final PathSE2Parameter m_parameter;

    final PathSE2Point m_point;
    /** Time we achieve this state. */
    final double m_timeS;
    /** Instantaneous pathwise velocity, m/s. */
    final double m_velocityM_S;
    /**
     * Pathwise acceleration for the timespan after this state, m/s^2. It's computed
     * by looking at the velocity of the next state, and the distance to get there.
     */
    final double m_accelM_S_S;

    public TimedStateSE2(
            PathSE2Parameter parameter,
            PathSE2Point point,
            double t,
            double velocity,
            double acceleration) {
        m_parameter = parameter;
        m_point = point;
        m_timeS = t;
        m_velocityM_S = velocity;
        m_accelM_S_S = acceleration;
    }

    public PathSE2Point point() {
        return m_point;
    }

    /** Time we achieve this state. */
    public double getTimeS() {
        return m_timeS;
    }

    /** Instantaneous pathwise velocity, m/s. */
    public double velocityM_S() {
        return m_velocityM_S;
    }

    /** Instantaneous pathwise (not centripetal) acceleration, m/s^2. */
    public double acceleration() {
        return m_accelM_S_S;
    }

    @Override
    public String toString() {
        return String.format("state %s, time %5.3f, vel %5.3f, acc %5.3f",
                m_point,
                m_timeS,
                m_velocityM_S,
                m_accelM_S_S);
    }

    /** Translation only, ignores rotation */
    public double distanceCartesian(TimedStateSE2 other) {
        return m_point.distanceCartesian(other.m_point);
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof TimedStateSE2)) {
            if (DEBUG)
                System.out.println("wrong type");
            return false;
        }
        TimedStateSE2 ts = (TimedStateSE2) other;
        if (!m_point.equals(ts.m_point)) {
            if (DEBUG)
                System.out.println("wrong state");
            return false;
        }
        if (!Math100.epsilonEquals(m_timeS, ts.m_timeS)) {
            if (DEBUG)
                System.out.println("wrong time");
            return false;
        }
        return true;
    }
}
