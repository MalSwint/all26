package org.team100.lib.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolator;

/**
 * Linear interpolation.
 * 
 * This is certainly unrealistic, don't use it for large differences.
 */
public class FiringSolutionInterpolator implements Interpolator<FiringSolution> {

    @Override
    public FiringSolution interpolate(FiringSolution a, FiringSolution b, double t) {
        return new FiringSolution(
                MathUtil.interpolate(a.range(), b.range(), t),
                MathUtil.interpolate(a.tof(), b.tof(), t),
                MathUtil.interpolate(a.targetElevation(), b.targetElevation(), t));
    }
}