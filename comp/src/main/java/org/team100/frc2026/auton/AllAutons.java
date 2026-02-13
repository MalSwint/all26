package org.team100.frc2026.auton;

import org.team100.frc2026.robot.Machinery;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.config.AutonChooser;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Populates the Auton chooser with all available autons.
 * 
 * It's a good idea to instantiate them all here, even if you're not using them
 * all, even if they're just in development, so they don't rot.
 * 
 * In 2026, the field is rotationally symmetric, so there's no need to fill out
 * the "Alliance" part of the annotated command.
 */
public class AllAutons {
    private final AutonChooser m_autonChooser;

    public AllAutons(Machinery machinery, ControllerSE2 controller) {
        m_autonChooser = new AutonChooser();
        LoggerFactory log = Logging.instance().rootLogger.name("Auton");
        m_autonChooser.add("Do Nothing",
                new AnnotatedCommand(
                        new DoNothing().withName("Nothing from right bump"),
                        null,
                        StartingPositions.RIGHT_BUMP));
        m_autonChooser.add("Right Trench Leave",
                AnnotatedCommand.eitherAlliance(
                        new RightTrenchLeave().get(
                                log,
                                machinery.m_swerveKinodynamics,
                                controller,
                                machinery),
                        StartingPositions.RIGHT_TRENCH));
    }

    public Command get() {
        AnnotatedCommand annotatedCommand = getAnnotated();
        if (annotatedCommand == null)
            return null;
        return annotatedCommand.command();
    }

    public AnnotatedCommand getAnnotated() {
        return m_autonChooser.get();
    }

    public void close() {
        m_autonChooser.close();
    }

}
