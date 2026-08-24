package frc.robot.subsystems.position;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.PositionControlConstants;
import frc.robot.utils.TelemetryManager;

public abstract class PositionControlSystem extends SubsystemBase {

    protected TalonFX m_positionControlMotor;
    protected String m_motorName;
    protected PositionControlConstants m_constants;

    // Reuse a single MotionMagicVoltage request object to avoid allocations
    private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0);

    // Cached StatusSignals -- avoids blocking CAN reads on every getValueAsDouble() call
    protected StatusSignal<Angle>   m_positionSignal;
    protected StatusSignal<Voltage> m_voltageSignal;
    protected StatusSignal<Current> m_statorCurrentSignal;
    protected StatusSignal<Current> m_supplyCurrentSignal;

    // Periodic loop counter for rate-limiting telemetry
    private int m_periodicCounter = 0;
    private static final int TELEMETRY_PERIOD = 5; // Update telemetry every 5 loops (100ms)

    // Zeroing state. Mechanisms that home against a hard stop have no idea where they are until
    // they have done so, and commanding a position before then can drive them into the frame.
    private boolean m_hasZeroed = false;

    public PositionControlSystem(int canId) {
        this(canId, "Unnamed Position Control System");
    }
    public PositionControlSystem(int canId, String name) {
        m_motorName = name;
        m_positionControlMotor = new TalonFX(canId);
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake);
        m_positionControlMotor.getConfigurator().apply(motorOutputConfigs);

        m_positionControlMotor.setPosition(0.0);
        m_positionControlMotor.optimizeBusUtilization();

        // Cache StatusSignal objects so periodic() can batch-refresh them in one CAN transaction
        m_positionSignal      = m_positionControlMotor.getPosition();
        m_voltageSignal       = m_positionControlMotor.getMotorVoltage();
        m_statorCurrentSignal = m_positionControlMotor.getStatorCurrent();
        m_supplyCurrentSignal = m_positionControlMotor.getSupplyCurrent();

        // Position signal needs to be fresh enough for isAtSetpoint() checks in commands.
        // 50 Hz keeps it at the full robot loop rate; telemetry signals can run slower.
        m_positionSignal.setUpdateFrequency(50);
        m_voltageSignal.setUpdateFrequency(10);
        m_statorCurrentSignal.setUpdateFrequency(10);
        m_supplyCurrentSignal.setUpdateFrequency(10);
    }

    protected void setInverted(boolean inverted) {
        MotorOutputConfigs motorConfig = new MotorOutputConfigs();
        motorConfig.Inverted = inverted ? 
            InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive;
        m_positionControlMotor.getConfigurator().apply(motorConfig);
    }

    public void setVoltage(double volts) {
        m_positionControlMotor.setVoltage(volts);
    }

    public void resetEncoder() {
        // This sets the internal rotor position to 0
        m_positionControlMotor.setPosition(0);
    }

    /** Returns stator current using the cached signal (no extra CAN read). */
    public double getStatorCurrent() {
        return m_statorCurrentSignal.getValueAsDouble();
    }

    // -------------------------------------------------------------------------
    // Hard-stop zeroing support
    //
    // A relative encoder reads 0 wherever the mechanism happened to be sitting at
    // power-on. For anything with a hard stop, the fix is to drive gently into that
    // stop, detect the stall, and declare that position known. These are the
    // primitives; the state machine that sequences them lives in a command -- see
    // ZeroExamplePositionSubsystem and the add-zeroing-command skill.
    // -------------------------------------------------------------------------

    /**
     * Records that this mechanism now knows where it is.
     *
     * <p>Only call this after the hard stop was actually reached. Calling it on a timeout marks a
     * mechanism as trustworthy when its encoder is meaningless, which is worse than not zeroing at
     * all -- every later setpoint will be off by the same unknown amount.
     */
    public void markZeroed() {
        m_hasZeroed = true;
    }

    /** Marks the mechanism as needing to re-zero. */
    public void clearZeroed() {
        m_hasZeroed = false;
    }

    /** True once this mechanism has successfully found its hard stop. */
    public boolean hasZeroed() {
        return m_hasZeroed;
    }

    /**
     * Writes the encoder reading that corresponds to the hard stop, then marks the mechanism zeroed.
     *
     * <p>Pass the mechanism position of the stop itself. That is often 0, but a mechanism that
     * homes against its fully-extended stop should pass its extended travel instead.
     *
     * @param mechanismInches position, in mechanism units, of the hard stop just reached
     */
    public void setEncoderToHardStop(double mechanismInches) {
        m_positionControlMotor.setPosition(inchesToRotations(mechanismInches));
        markZeroed();
    }

    public void setDefaultConstants(PositionControlConstants constants ){
        m_constants = constants;
        // Configure Motion Magic settings
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = constants.kCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = constants.kAcceleration;
        m_positionControlMotor.getConfigurator().apply(motionMagicConfigs);

        // Configure PIDF settings
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = constants.kP;
        slot0Configs.kI = constants.kI;
        slot0Configs.kD = constants.kD;
        slot0Configs.kV = constants.kV;
        m_positionControlMotor.getConfigurator().apply(slot0Configs);

        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(constants.kSupplyCurrentLimit)
            .withStatorCurrentLimit(constants.kStatorCurrentLimit);
        m_positionControlMotor.getConfigurator().apply(limitConfigs);       
    }

    /**
     * Converts mechanism travel to motor rotations using {@code kInchesPerRotation}.
     *
     * <p>Note this is a different number from {@code kGearRatio}: kInchesPerRotation describes
     * linear travel per motor turn, kGearRatio describes motor turns per mechanism turn and is
     * only used for the angle telemetry readout.
     */
    protected double inchesToRotations(double inches) {
        return inches / m_constants.kInchesPerRotation;
    }

    /**
     * Converts motor rotations to inches
     */
    protected double rotationsToInches(double rotations) {
        return rotations * m_constants.kInchesPerRotation;
    }

    public void setRotation(double motorRotations) {
        m_positionControlMotor.setControl(m_motionMagicRequest.withPosition(motorRotations));
    }

    public boolean isAtSetpoint(double targetRotations, double toleranceRotations) {
        // Uses cached signal -- no extra CAN read
        double currentRotations = m_positionSignal.getValueAsDouble();
        return Math.abs(currentRotations - targetRotations) < toleranceRotations;
    }

    public void stopMotor() {
        m_positionControlMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // A TalonFX that browns out and reboots comes back with its encoder reset to 0, silently
        // invalidating a zero we established earlier. Checking the reset flag here is the only way
        // to notice; without it the mechanism keeps running against a bogus reference all match.
        if (m_hasZeroed && m_positionControlMotor.hasResetOccurred()) {
            clearZeroed();
            DriverStation.reportWarning(
                m_motorName + " lost power and must re-zero before it can be commanded.", false);
        }

        m_periodicCounter++;

        // Only update telemetry every TELEMETRY_PERIOD loops to reduce CAN reads and NT writes
        if (m_periodicCounter % TELEMETRY_PERIOD != 0) return;

        // Refresh all cached signals in ONE CAN transaction instead of 4 separate reads
        BaseStatusSignal.refreshAll(
            m_positionSignal,
            m_voltageSignal,
            m_statorCurrentSignal,
            m_supplyCurrentSignal
        );

        // Read from cached signals -- no additional CAN traffic
        double rotations = m_positionSignal.getValueAsDouble();
        double systemAngle = (rotations * 360.0) / m_constants.kGearRatio;
        double currentInches = rotationsToInches(rotations);

        TelemetryManager.putNumber(m_motorName + " Voltage", m_voltageSignal.getValueAsDouble());
        TelemetryManager.putNumber(m_motorName + " Angle", systemAngle);
        TelemetryManager.putNumber(m_motorName + " Position", currentInches);
        TelemetryManager.putNumber(m_motorName + " Motor Stator Current", m_statorCurrentSignal.getValueAsDouble());
        TelemetryManager.putNumber(m_motorName + " Motor Supply Current", m_supplyCurrentSignal.getValueAsDouble());
    }

    public static PositionControlSystem getInstance() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getInstance'");
    }
}
