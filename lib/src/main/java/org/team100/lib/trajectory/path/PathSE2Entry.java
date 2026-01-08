package org.team100.lib.trajectory.path;

/**
 * Contains the "key" (spline, s value) and "value" (waypoint, curvature) of a
 * path sample
 */
public record PathSE2Entry(PathSE2Parameter parameter, PathSE2Point point) {
}
