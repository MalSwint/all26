package org.team100.frc2026;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.ctre.Falcon6Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.profile.r1.TrapezoidIncrementalProfile;
import org.team100.lib.reference.r1.IncrementalProfileReferenceR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.sensor.position.absolute.EncoderDrive;
import org.team100.lib.sensor.position.absolute.RotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.sim.SimulatedRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.wpi.AS5048RotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.OnboardAngularPositionServo;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeExtend extends SubsystemBase {
    private final AngularPositionServo m_servo;

    public IntakeExtend(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        SimulatedBareMotor climberMotor = new SimulatedBareMotor(log, 600);

        IncrementalProfile profile = new TrapezoidIncrementalProfile(log, 1, 2, 0.05);
        ProfileReferenceR1 ref = new IncrementalProfileReferenceR1(log, () -> profile, 0.05, 0.05);
        PIDFeedback feedback = new PIDFeedback(log, 5, 0, 0, false, 0.05, 0.1);

        IncrementalBareEncoder encoder = climberMotor.encoder();
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(log, encoder, 1);

        RotaryMechanism climberMech = new RotaryMechanism(
            log, climberMotor, sensor, 1,
            Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        m_servo = new OnboardAngularPositionServo(log, climberMech, ref, feedback);
    }



    @Override
    public void periodic() {
        m_servo.periodic();
    }

    public Command goToExtendedPosition() {
        return startRun(
                () -> reset(),
                () -> setAngle(Math.PI / 2));
    }

    public Command goToRetractedPosition() {
        return startRun(
                () -> reset(),
                () -> setAngle(0));
    }


    public Command stop() {
        return run(this::stopServo);
    }

    public void stopServo() {
        m_servo.stop();
    }

    private void reset() {
        m_servo.reset();
    }

    /** Use a profile to set the position. */
    private void setAngle(double value) {
        m_servo.setPositionProfiled(value, 0);
    }

}
