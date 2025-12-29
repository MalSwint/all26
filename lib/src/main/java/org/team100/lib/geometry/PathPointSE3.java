package org.team100.lib.geometry;

/**
 * Represents a point on a path in SE(3) (3d space with rotation).
 * 
 * Includes a WaypointSE2, heading rate, and curvature.
 */
public class PathPointSE3 {
    /** Pose and course. */
    private final WaypointSE3 m_waypoint;
    /**
     * Unit normal in the direction of the center of curvature.
     * https://en.wikipedia.org/wiki/Center_of_curvature
     */
    private final DirectionR3 m_N;
    /**
     * Curvature (1/radius of osculating circle), rad/m
     * https://en.wikipedia.org/wiki/Osculating_circle
     */
    private final double m_k;

    // TODO: the other fields
    //
    // how to represent curvature?

    /**
     * @param waypoint
     * @param N        normal towards center of curvature
     * @param k        curvature
     */
    public PathPointSE3(WaypointSE3 waypoint, DirectionR3 N, double k) {
        m_waypoint = waypoint;
        m_N = N;
        m_k = k;
    }

    public WaypointSE3 waypoint() {
        return m_waypoint;
    }

}
