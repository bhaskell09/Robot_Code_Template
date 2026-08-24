package frc.robot.subsystems.position;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ExamplePositionSubsystemPositions;

/**
 * Reference position-controlled subsystem -- an elevator, arm, wrist, hood, or deployer.
 *
 * <p>Like the velocity example, nearly everything lives in {@link PositionControlSystem}. What a
 * concrete position subsystem adds is the <b>unit boundary</b>: the base class thinks in motor
 * rotations, while drivers, commands, and dashboards think in inches (or degrees). Converting in
 * exactly one place -- here -- is what keeps a rotations value from being passed to something
 * expecting inches.
 *
 * <p><b>Two things worth copying deliberately:</b>
 *
 * <p>1. {@code setPosition()} clamps against the soft limits before commanding. The clamp lives
 * here rather than in each command, so a new command physically cannot ask for a setpoint outside
 * the mechanism's travel.
 *
 * <p>2. The setpoint check is named {@code isAtInches}, <b>not</b> {@code isAtSetpoint}. The base
 * class already has an {@code isAtSetpoint(double, double)} that compares rotations. Declaring an
 * inches version at that same signature would override it silently, and any caller holding a
 * base-class reference would get inches semantics while reading code that says rotations. A
 * separate name makes the unit visible at every call site.
 *
 * <p>Delete this file once you have real mechanisms.
 */
public class ExamplePositionSubsystem extends PositionControlSystem {

    public ExamplePositionSubsystem() {
        super(Constants.ExamplePositionSubsystemConstants.kCanId, "ExamplePositionSubsystem");
        setDefaultConstants(Constants.ExamplePositionSubsystemConstants);
        setInverted(false);
    }

    /**
     * Drives the mechanism to a position, in inches, using Motion Magic.
     *
     * <p>Clamped to the mechanism's travel, so an out-of-range request is capped instead of driving
     * into a hard stop at full authority.
     *
     * @param inches target position in mechanism inches
     */
    public void setPosition(double inches) {
        double clamped = MathUtil.clamp(
            inches,
            ExamplePositionSubsystemPositions.MIN_POSITION,
            ExamplePositionSubsystemPositions.MAX_POSITION);
        setRotation(inchesToRotations(clamped));
    }

    /** Current mechanism position in inches, from the cached signal (no extra CAN read). */
    public double getPositionInches() {
        return rotationsToInches(m_positionSignal.getValueAsDouble());
    }

    /**
     * True when the mechanism is within toleranceInches of targetInches.
     *
     * <p>Deliberately not called isAtSetpoint -- see the class Javadoc.
     */
    public boolean isAtInches(double targetInches, double toleranceInches) {
        return Math.abs(getPositionInches() - targetInches) < toleranceInches;
    }

    /**
     * Applies the open-loop voltage used while homing.
     *
     * <p>Zeroing bypasses Motion Magic entirely: the whole point is to move without a trusted
     * reference, which closed-loop position control cannot do.
     */
    public void setZeroingOutput(double volts) {
        setVoltage(volts);
    }
}
