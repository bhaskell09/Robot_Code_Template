package frc.robot.subsystems.velocity;

import frc.robot.constants.Constants;

/**
 * Reference velocity-controlled subsystem -- a flywheel, roller, feeder, or indexer.
 *
 * <p><b>This whole file is the subsystem.</b> That is the point of the pattern: PID configuration,
 * current limits, StatusSignal caching, batched CAN refreshes, and telemetry all live in
 * {@link VelocityControlSystem}, so a new mechanism costs three lines. If you find yourself copying
 * logic into a subclass, it probably belongs in the base class instead.
 *
 * <p><b>To add your own:</b> CAN ID into {@code RobotMap}, a {@code VelocityControlConstants} into
 * {@code Constants}, copy this file, wire it up in {@code RobotContainer}. The
 * {@code add-motor-subsystem} skill walks through it.
 *
 * <p><b>Inversion belongs here</b>, in the constructor, and nowhere else. A command that negates a
 * speed works right up until a second command drives the same motor and forgets to -- see the note
 * in {@code RunExampleVelocitySubsystem}.
 *
 * <p>Delete this file once you have real mechanisms.
 */
public class ExampleVelocitySubsystem extends VelocityControlSystem {

    public ExampleVelocitySubsystem() {
        super(Constants.ExampleVelocitySubsystemConstants.kCanId, "ExampleVelocitySubsystem");
        setDefaultConstants(Constants.ExampleVelocitySubsystemConstants);
        setInverted(false);
    }
}
