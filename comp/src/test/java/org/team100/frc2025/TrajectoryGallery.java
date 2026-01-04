package org.team100.frc2025;

import java.util.Collection;
import java.util.List;
import java.util.stream.Stream;

import org.jfree.data.xy.VectorSeries;
import org.junit.jupiter.api.Test;
import org.team100.frc2025.Swerve.Auto.GoToCoralStation;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.field.FieldConstants;
import org.team100.lib.field.FieldConstants.CoralStation;
import org.team100.lib.field.FieldConstants.ReefPoint;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.trajectory.TrajectorySE2;
import org.team100.lib.trajectory.TrajectorySE2ToVectorSeries;
import org.team100.lib.util.ChartUtil;

import edu.wpi.first.math.geometry.Pose2d;

/** Show some trajectories from 2025 in a vector series chart. */
public class TrajectoryGallery {
    LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
    SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest(log);

    @Test
    void testGoToCoralStation1() {
        List<VectorSeries> right = series(CoralStation.RIGHT, ReefPoint.F, ScoringLevel.L4);
        ChartUtil.plotOverlay(right);
    }

    @Test
    void testGoToCoralStation2() {
        List<VectorSeries> left = series(CoralStation.LEFT, ReefPoint.I, ScoringLevel.L4);
        ChartUtil.plotOverlay(left);
    }

    @Test
    void testGoToCoralStation3() {
        List<VectorSeries> iLeft = series(CoralStation.LEFT, ReefPoint.I, ScoringLevel.L4);
        List<VectorSeries> kLeft = series(CoralStation.LEFT, ReefPoint.K, ScoringLevel.L4);
        List<VectorSeries> lLeft = series(CoralStation.LEFT, ReefPoint.L, ScoringLevel.L4);
        List<VectorSeries> fRight = series(CoralStation.RIGHT, ReefPoint.F, ScoringLevel.L4);
        List<VectorSeries> dRight = series(CoralStation.RIGHT, ReefPoint.D, ScoringLevel.L4);
        List<VectorSeries> cRight = series(CoralStation.RIGHT, ReefPoint.C, ScoringLevel.L4);
        ChartUtil.plotOverlay(Stream.of(iLeft, kLeft, lLeft, fRight, dRight, cRight)
                .flatMap(Collection::stream).toList());
    }

    private List<VectorSeries> series(CoralStation coralStation, ReefPoint reefPoint, ScoringLevel scoringLevel) {
        GoToCoralStation toStation = new GoToCoralStation(log, swerveKinodynamics, coralStation, 0.5);
        // the start is the goal from the previous maneuver
        Pose2d start = FieldConstants.makeGoal(scoringLevel, reefPoint);
        TrajectorySE2 trajectory = toStation.apply(start);
        return new TrajectorySE2ToVectorSeries(0.1).convert(trajectory);
    }

}
