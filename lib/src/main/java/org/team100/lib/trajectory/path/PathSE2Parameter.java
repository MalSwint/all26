package org.team100.lib.trajectory.path;

import org.team100.lib.trajectory.spline.SplineSE2;

/** A point on a spline. */
public record PathSE2Parameter(SplineSE2 spline, double s) {
}