package org.team100.lib.config;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command with annotations that are checked against ground truth while
 * disabled.
 */
public interface AnnotatedCommand {

    /**
     * command to run
     */
    Command command();

    /**
     * red or blue, null if it works for both.
     */
    default Alliance alliance() {
        return null;
    }

    /**
     * starting pose, null if it doesn't matter.
     */
    default Pose2d start() {
        return null;
    }
}
