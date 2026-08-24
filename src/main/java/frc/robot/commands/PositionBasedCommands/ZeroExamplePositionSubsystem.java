package frc.robot.commands.PositionBasedCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ExamplePositionSubsystemZeroingConstants;
import frc.robot.subsystems.position.ExamplePositionSubsystem;

/**
 * Reference hard-stop zeroing command.
 *
 * <p>A relative encoder reads zero wherever the mechanism happened to be sitting when the robot
 * powered on. Homing fixes that: drive gently into a known physical stop, notice the stall, and
 * declare that position known.
 *
 * <p><b>Two phases.</b> Retract briefly first, then approach. Backing off matters because the
 * mechanism may already be resting against the stop at power-on -- in which case an immediate
 * approach reads stall current instantly and "zeroes" without moving, which happens to be correct
 * here but is indistinguishable from a jam. Backing off first means the stall we detect is always
 * one we caused, and it guarantees a consistent approach direction so backlash lands the same way
 * every time.
 *
 * <p><b>Detection is debounced.</b> Raw stator current spikes on direction changes and on any
 * sticky spot in the travel. An undebounced threshold latches onto the first of those and zeroes
 * the mechanism in the middle of its range. Requiring the current to stay high for a continuous
 * window rejects transients while still stopping well before anything is damaged.
 *
 * <p><b>On timeout, nothing is marked zeroed.</b> That is the single most important line in this
 * file. A mechanism wrongly marked as zeroed is worse than one that admits it does not know where
 * it is: every setpoint afterward is off by the same unknown amount, and the failure shows up as
 * mysterious mechanical misbehavior rather than as an error anyone can act on.
 */
public class ZeroExamplePositionSubsystem extends Command {

    private enum Phase { RETRACTING, APPROACHING }

    private final ExamplePositionSubsystem m_subsystem;
    private final Timer m_timer = new Timer();
    private final Debouncer m_stallDebouncer = new Debouncer(
        ExamplePositionSubsystemZeroingConstants.STALL_DEBOUNCE_SECONDS, Debouncer.DebounceType.kRising);

    private Phase m_phase;
    private boolean m_foundHardStop;
    private boolean m_timedOut;

    public ZeroExamplePositionSubsystem(ExamplePositionSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        m_phase = Phase.RETRACTING;
        m_foundHardStop = false;
        m_timedOut = false;
        m_stallDebouncer.calculate(false); // clear any state left from a previous run
        m_subsystem.clearZeroed();
        m_timer.restart();
    }

    @Override
    public void execute() {
        if (m_timer.hasElapsed(ExamplePositionSubsystemZeroingConstants.TIMEOUT_SECONDS)) {
            m_timedOut = true;
            return;
        }

        switch (m_phase) {
            case RETRACTING -> {
                m_subsystem.setZeroingOutput(ExamplePositionSubsystemZeroingConstants.RETRACT_VOLTAGE);
                if (m_timer.hasElapsed(ExamplePositionSubsystemZeroingConstants.RETRACT_SECONDS)) {
                    m_phase = Phase.APPROACHING;
                }
            }
            case APPROACHING -> {
                m_subsystem.setZeroingOutput(ExamplePositionSubsystemZeroingConstants.ZEROING_VOLTAGE);
                boolean stalled = m_subsystem.getStatorCurrent()
                    > ExamplePositionSubsystemZeroingConstants.STALL_CURRENT_THRESHOLD;
                if (m_stallDebouncer.calculate(stalled)) {
                    m_foundHardStop = true;
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return m_foundHardStop || m_timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setZeroingOutput(0.0);

        if (m_foundHardStop && !interrupted) {
            m_subsystem.setEncoderToHardStop(
                ExamplePositionSubsystemZeroingConstants.HARD_STOP_INCHES);
            return;
        }

        // Reached only on timeout or interruption. markZeroed() is intentionally NOT called -- the
        // mechanism stays untrusted, and SetExamplePositionSubsystem will refuse to move it until
        // zeroing succeeds. Loud and stationary beats quiet and wrong.
        if (m_timedOut) {
            DriverStation.reportWarning(
                "Zeroing timed out before reaching the hard stop. Mechanism is NOT zeroed.", false);
        }
    }
}
