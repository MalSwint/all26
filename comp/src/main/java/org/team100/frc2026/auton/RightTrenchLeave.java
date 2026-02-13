package org.team100.frc2026.auton;

import java.util.List;
import java.util.function.Function;

import org.team100.frc2026.robot.Machinery;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.se2.commands.DriveWithTrajectoryFunction;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.TrajectorySE2;
import org.team100.lib.trajectory.TrajectorySE2Factory;
import org.team100.lib.trajectory.TrajectorySE2Planner;
import org.team100.lib.trajectory.constraint.TimingConstraint;
import org.team100.lib.trajectory.constraint.TimingConstraintFactory;
import org.team100.lib.trajectory.path.PathSE2Factory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Drives a short distance from the right trench starting point */
public class RightTrenchLeave {

    public Command get(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            ControllerSE2 controller,
            Machinery machinery) {
        LoggerFactory log = parent.name("RightTrenchLeave");
        List<TimingConstraint> constraints = new TimingConstraintFactory(kinodynamics).auto(log.type(this));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);

        Function<Pose2d, TrajectorySE2> trajectoryFn = (p) -> {
            List<WaypointSE2> waypoints = List.of(
                    new WaypointSE2(
                            p,
                            new DirectionSE2(1, 0, 0),
                            1),
                    new WaypointSE2(
                            new Pose2d(),
                            new DirectionSE2(1, 0, 0),
                            1));
            return planner.restToRest(waypoints);
        };

        DriveWithTrajectoryFunction navigator = new DriveWithTrajectoryFunction(
                log,
                machinery.m_drive,
                controller,
                machinery.m_trajectoryViz,
                trajectoryFn);

        return navigator
                .until(navigator::isDone);

    }

}
