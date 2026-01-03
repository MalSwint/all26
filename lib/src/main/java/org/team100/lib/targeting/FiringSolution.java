package org.team100.lib.targeting;

/**
 * Solution to a targeting problem.
 * 
 * @param range           in meters
 * @param tof             time of flight in seconds
 * @param targetElevation path angle at the target
 */
public record FiringSolution(double range, double tof, double targetElevation) {
}