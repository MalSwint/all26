package org.team100.lib.trajectory.path.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE3;
import org.team100.lib.geometry.WaypointSE3;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class HolonomicSplineSE3Test implements Timeless {
    private static final double DELTA = 0.001;

    @Test
    void testLinear() {
        WaypointSE3 w0 = new WaypointSE3(
                new Pose3d(
                        new Translation3d(),
                        new Rotation3d()),
                new DirectionSE3(1, 0, 0, 0, 0, 0), 1);
        WaypointSE3 w1 = new WaypointSE3(
                new Pose3d(
                        new Translation3d(1, 0, 0),
                        new Rotation3d()),
                new DirectionSE3(1, 0, 0, 0, 0, 0), 1);
        HolonomicSplineSE3 spline = new HolonomicSplineSE3(w0, w1);
        Translation3d t = spline.sample(0).waypoint().pose().getTranslation();
        assertEquals(0, t.getX(), DELTA);
        t = spline.sample(1).waypoint().pose().getTranslation();
        assertEquals(1, t.getX(), DELTA);
    }

    @Test
    void testDump() {
        WaypointSE3 w0 = new WaypointSE3(
                new Pose3d(
                        new Translation3d(),
                        new Rotation3d()),
                new DirectionSE3(1, 0, 0, 0, 0, 0), 1);
        WaypointSE3 w1 = new WaypointSE3(
                new Pose3d(
                        new Translation3d(1, 1, 1),
                        new Rotation3d()),
                new DirectionSE3(0, 0, 1, 0, 0, 0), 1);
        HolonomicSplineSE3 spline = new HolonomicSplineSE3(w0, w1);
        spline.dump();
    }

}
