package org.team100.lib.trajectory.path.spline;

import org.team100.lib.geometry.DirectionR3;
import org.team100.lib.geometry.DirectionSE3;
import org.team100.lib.geometry.PathPointSE3;
import org.team100.lib.geometry.WaypointSE3;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Holonomic spline in the SE(3) manifold, the space of Pose3d.
 * 
 * Internally this is five one-dimensional splines (x, y, z, phi, theta), with
 * respect to a parameter [0,1].
 * 
 * Elsewhere, we use Rotation3d to describe spherical angles, by applying the
 * rotation to the translation (1,0,0), as Pose3d does. The coordinate system
 * there is "roll pitch yaw" from the reference, not "spherical coordinates"
 * (with a polar angle etc).
 */
public class HolonomicSplineSE3 {
    private static final boolean DEBUG = false;
    // curvature measurement performance scales with sample count so make it kinda
    // low. most splines go between 0.5 and 5 meters so this is steps of 2 to 20 cm.
    private static final int SAMPLES = 25;

    // these are for position
    private final SplineR1 m_x;
    private final SplineR1 m_y;
    private final SplineR1 m_z;

    // these are for heading
    private final SplineR1 m_roll;
    private final SplineR1 m_pitch;
    private final SplineR1 m_yaw;
    private final Rotation2d m_roll0;
    private final Rotation2d m_pitch0;
    private final Rotation2d m_yaw0;
    private final Rotation3d m_heading0;

    public HolonomicSplineSE3(WaypointSE3 p0, WaypointSE3 p1) {
        this(p0, p1, 1.2, 1.2);
    }

    public HolonomicSplineSE3(WaypointSE3 p0, WaypointSE3 p1, double mN0, double mN1) {
        double scale0 = mN0 * p0.pose().getTranslation().getDistance(p1.pose().getTranslation());
        double scale1 = mN1 * p0.pose().getTranslation().getDistance(p1.pose().getTranslation());

        DirectionSE3 course0 = p0.course();
        DirectionSE3 course1 = p1.course();

        double x0 = p0.pose().getTranslation().getX();
        double x1 = p1.pose().getTranslation().getX();
        // first derivatives are just the course
        double dx0 = course0.x * scale0;
        double dx1 = course1.x * scale1;
        // second derivatives are zero at the ends
        double ddx0 = 0;
        double ddx1 = 0;

        double y0 = p0.pose().getTranslation().getY();
        double y1 = p1.pose().getTranslation().getY();
        // first derivatives are just the course
        double dy0 = course0.y * scale0;
        double dy1 = course1.y * scale1;
        // second derivatives are zero at the ends
        double ddy0 = 0;
        double ddy1 = 0;

        double z0 = p0.pose().getTranslation().getZ();
        double z1 = p1.pose().getTranslation().getZ();
        // first derivatives are just the course
        double dz0 = course0.z * scale0;
        double dz1 = course1.z * scale1;
        // second derivatives are zero at the ends
        double ddz0 = 0;
        double ddz1 = 0;

        m_x = SplineR1.get(x0, x1, dx0, dx1, ddx0, ddx1);
        m_y = SplineR1.get(y0, y1, dy0, dy1, ddy0, ddy1);
        m_z = SplineR1.get(z0, z1, dz0, dz1, ddz0, ddz1);

        m_heading0 = p0.pose().getRotation();
        m_roll0 = new Rotation2d(p0.pose().getRotation().getX());
        m_pitch0 = new Rotation2d(p0.pose().getRotation().getY());
        m_yaw0 = new Rotation2d(p0.pose().getRotation().getZ());

        Rotation3d headingDelta = p1.pose().getRotation().minus(p0.pose().getRotation());
        double rollDelta = headingDelta.getX();
        double pitchDelta = headingDelta.getY();
        double yawDelta = headingDelta.getZ();

        // first derivative is the average
        double droll0 = rollDelta * mN0;
        double droll1 = rollDelta * mN1;
        // second derivatives are zero at the ends
        double ddroll0 = 0;
        double ddroll1 = 0;
        m_roll = SplineR1.get(0.0, rollDelta, droll0, droll1, ddroll0, ddroll1);

        // first derivative is the average
        double dpitch0 = pitchDelta * mN0;
        double dpitch1 = pitchDelta * mN1;
        // second derivatives are zero at the ends
        double ddpitch0 = 0;
        double ddpitch1 = 0;
        m_pitch = SplineR1.get(0.0, pitchDelta, dpitch0, dpitch1, ddpitch0, ddpitch1);

        // first derivative is the average
        double dyaw0 = yawDelta * mN0;
        double dyaw1 = yawDelta * mN1;
        // second derivatives are zero at the ends
        double ddyaw0 = 0;
        double ddyaw1 = 0;
        m_yaw = SplineR1.get(0.0, yawDelta, dyaw0, dyaw1, ddyaw0, ddyaw1);

    }

    /**
     * TODO: remove the "1" scale here
     * 
     * @param s [0,1]
     */
    public PathPointSE3 sample(double s) {
        return new PathPointSE3(new WaypointSE3(
                new Pose3d(new Translation3d(x(s), y(s), z(s)), getHeading(s)),
                getCourse(s), 1),
                getN(s),
                getK(s));
    }

    DirectionR3 getN(double s) {
        return null;
    }

    double getK(double s) {
        return 0;
    }

    double x(double s) {
        return m_x.getPosition(s);
    }

    double y(double s) {
        return m_y.getPosition(s);
    }

    double z(double s) {
        return m_z.getPosition(s);
    }

    public DirectionSE3 getCourse(double S) {
        double dx = dx(S);
        double dy = dy(S);
        double dz = dz(S);
        double dr = droll(S);
        double dp = dpitch(S);
        double dyaw = dyaw(S);
        return new DirectionSE3(dx, dy, dz, dr, dp, dyaw);
    }

    protected Rotation3d getHeading(double s) {
        Rotation3d delta = new Rotation3d(
                m_roll.getPosition(s), m_pitch.getPosition(s), m_yaw.getPosition(s));
        return m_heading0.rotateBy(delta);
    }

    double dx(double s) {
        return m_x.getVelocity(s);
    }

    double dy(double s) {
        return m_y.getVelocity(s);
    }

    double dz(double s) {
        return m_z.getVelocity(s);
    }

    double droll(double s) {
        return m_roll.getVelocity(s);
    }

    double dpitch(double s) {
        return m_pitch.getVelocity(s);
    }

    double dyaw(double s) {
        return m_yaw.getVelocity(s);
    }

    /**
     * Velocity is the change in position per parameter, p: ds/dp (meters per p).
     * Since p is not time, it is not "velocity" in the usual sense.
     */
    protected double getVelocity(double t) {
        double dx = dx(t);
        double dy = dy(t);
        double dz = dy(t);
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    /**
     * Print samples for testing.
     * 
     * Uses the format of https://miabellaai.net/index.html, paste the data into the
     * box near the bottom of the page, to see a 3d scatterplot.
     */
    public void dump() {
        System.out.println("HolonomicSplineSE3;");
        System.out.println("::X::Y::Z;");
        for (double s = 0; s <= 1; s += 0.05) {
            System.out.printf("A::%5.3f::%5.3f::%5.3f::1::50::A::1::0::0::0::0;\n", x(s), y(s), z(s));
        }
    }

}
