package org.team100.lib.trajectory.constraint;

import org.team100.lib.trajectory.path.PathPointSE3;

public interface TimingConstraintSE3 {
    double maxV(PathPointSE3 state);

    double maxAccel(PathPointSE3 state, double velocityM_S);

    double maxDecel(PathPointSE3 state, double velocityM_S);

}
