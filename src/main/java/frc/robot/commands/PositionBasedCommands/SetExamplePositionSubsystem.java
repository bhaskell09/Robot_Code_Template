package frc.robot.commands.PositionBasedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ExamplePositionSubsystemPositions;
import frc.robot.subsystems.position.ExamplePositionSubsystem;

/**
 * Reference command for a position-controlled subsystem.
 *
 * <p>Sends the mechanism to a setpoint and finishes once it arrives.
 *
 * <p><b>Two details here are load-bearing.</b>
 *
 * <p>1. The setpoint is commanded once, in {@code initialize()}. Motion Magic generates and follows
 * the trapezoidal profile onboard the TalonFX, so re-commanding the same target every loop adds CAN
 * traffic and changes nothing. Move it to {@code execute()} only when the target genuinely changes
 * while the command runs -- a dashboard tunable, or a continuously tracked target.
 *
 * <p>2. {@code end()} is empty <b>on purpose</b>. See the comment there before changing it.
 */
public class SetExamplePositionSubsystem extends Command {

    private final ExamplePositionSubsystem m_subsystem;
    private final double m_targetInches;
    private final Timer m_timer = new Timer();

    /**
     * A mechanism that jams, mis-tunes, or loses its zero will never satisfy isFinished(). Without
     * a ceiling this command hangs forever -- and in autonomous, it hangs the rest of the routine
     * with it. Finishing late beats never finishing.
     */
    private static final double TIMEOUT_SECONDS = 3.0;

    public SetExamplePositionSubsystem(ExamplePositionSubsystem subsystem, double targetInches) {
        m_subsystem = subsystem;
        m_targetInches = targetInches;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        m_timer.restart();

        // If this mechanism homes against a hard stop, refuse to move until it has. Commanding a
        // position off an untrusted encoder means the mechanism goes somewhere nobody predicted,
        // which for anything with real travel means driving into the frame at full authority.
        if (!m_subsystem.hasZeroed()) {
            return;
        }

        m_subsystem.setPosition(m_targetInches);
    }

    @Override
    public boolean isFinished() {
        if (!m_subsystem.hasZeroed()) {
            return true; // nothing was commanded; do not hold the scheduler
        }
        return m_subsystem.isAtInches(
                   m_targetInches, ExamplePositionSubsystemPositions.POSITION_TOLERANCE)
            || m_timer.hasElapsed(TIMEOUT_SECONDS);
    }

    @Override
    public void end(boolean interrupted) {
        // Deliberately empty -- this is how the mechanism HOLDS its position.
        //
        // Motion Magic keeps servoing to the last commanded setpoint after the command ends, and
        // the motor is in Brake neutral mode, so doing nothing is what keeps an arm up.
        //
        // Do NOT add setPosition(0) or stopMotor() here. On a gravity-loaded mechanism, commanding
        // zero drives it to the floor at full authority and stopMotor() drops it. This is the most
        // common way to break a mechanism in this codebase.
        //
        // A "return to stow" variant is legitimate for mechanisms where a known safe position
        // exists and travelling there unattended is genuinely safe:
        //
        //     m_subsystem.setPosition(ExamplePositionSubsystemPositions.STOWED);
        //
        // Only do that when STOWED really is safe to reach from anywhere in the travel -- and note
        // it is a different behavior, not a tidier version of holding.
    }
}
