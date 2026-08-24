package frc.robot.commands.VelocityBasedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.TunableConstants;
import frc.robot.subsystems.velocity.VelocityControlSystem;

/**
 * Reference command for a velocity-controlled subsystem.
 *
 * <p>Runs a motor at a target RPM for as long as it is scheduled, then stops it.
 *
 * <p><b>Note the field type:</b> {@code VelocityControlSystem}, not the concrete subsystem. Any
 * velocity subsystem satisfies it, so this one command drives a flywheel, a roller, or a feeder
 * without modification -- and survives into next season when the mechanisms change but the base
 * class does not. Type against the concrete class only when you need a method it alone defines.
 *
 * <p><b>Where the target RPM comes from</b> is a real choice, so it is explicit here rather than
 * implied. With {@code useTunableConstants} true, the value is re-read from the dashboard every
 * loop and the constructor argument is ignored. With it false, the constructor argument is
 * authoritative. The 2026 code had commands that took an RPM argument and then silently overwrote
 * it from the dashboard on the first {@code execute()}, which made call sites read like lies.
 */
public class RunExampleVelocitySubsystem extends Command {

    private final VelocityControlSystem m_subsystem;
    private final double m_targetRPM;
    private final boolean m_useTunableConstants;

    /** Runs at a fixed RPM. */
    public RunExampleVelocitySubsystem(VelocityControlSystem subsystem, double targetRPM) {
        this(subsystem, targetRPM, false);
    }

    /**
     * @param subsystem            the motor to drive
     * @param targetRPM            target speed; ignored when useTunableConstants is true
     * @param useTunableConstants  true to read the speed live from the dashboard each loop
     */
    public RunExampleVelocitySubsystem(VelocityControlSystem subsystem, double targetRPM,
                                       boolean useTunableConstants) {
        m_subsystem = subsystem;
        m_targetRPM = targetRPM;
        m_useTunableConstants = useTunableConstants;

        // Declaring the requirement is what stops two commands from fighting over this motor.
        // Without it, both run at once and the motor obeys whichever wrote last -- a bug that looks
        // electrical and wastes an afternoon in the pit.
        addRequirements(m_subsystem);
    }

    @Override
    public void execute() {
        // Read the tunable HERE, not in the constructor. The constructor runs once at robot boot,
        // so a value captured there is frozen until the next deploy -- which defeats the purpose of
        // having it on the dashboard at all.
        double target = m_useTunableConstants
            ? TunableConstants.getExampleVelocityRPM()
            : m_targetRPM;

        // Never negate the speed here to correct a motor's direction. Inversion is a property of
        // how the motor is mounted, so it belongs in the subsystem constructor via setInverted().
        // Fix it in a command and every other command driving that motor has to remember to do the
        // same, and one of them eventually will not.
        m_subsystem.setSpeed(target);
    }

    @Override
    public void end(boolean interrupted) {
        // Safe to stop outright: a flywheel coasts down harmlessly. This is exactly the opposite of
        // a position mechanism, where cutting output can drop a loaded arm. See
        // SetExamplePositionSubsystem.end().
        m_subsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        // Never finishes on its own. Bound with whileTrue() it runs while the button is held; in an
        // auto routine, wrap it with .withTimeout() or compose it with the command that ends it.
        //
        // If instead you want a spin-up step inside a sequence, return:
        //     return m_subsystem.isAtSetpoint(target, TOLERANCE_RPM);
        return false;
    }
}
