package frc.robot.subsystems.velocity;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.VelocityControlConstants;
import frc.robot.utils.TelemetryManager;

public abstract class VelocityControlSystem extends SubsystemBase {

    protected TalonFX m_velocityControlMotor;
    protected String m_motorName;
    protected VelocityControlConstants m_constants;
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    // Cached StatusSignals -- avoids blocking CAN reads on every getValueAsDouble() call
    protected StatusSignal<AngularVelocity> m_velocitySignal;
    protected StatusSignal<Voltage> m_voltageSignal;
    protected StatusSignal<Current> m_statorCurrentSignal;
    protected StatusSignal<Current> m_supplyCurrentSignal;

    // Periodic loop counter for rate-limiting telemetry
    private int m_periodicCounter = 0;
    private static final int TELEMETRY_PERIOD = 5; // Update telemetry every 5 loops (100ms)


    public VelocityControlSystem(int canId) {
        this(canId, "Unnamed Velocity Control System");
    }
    public VelocityControlSystem(int canId, String name) {
        m_motorName = name;
        m_velocityControlMotor = new TalonFX(canId);

        // Ensure the motor is in Coast mode by default
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast);
        m_velocityControlMotor.getConfigurator().apply(motorOutputConfigs);
        m_velocityControlMotor.optimizeBusUtilization();

        // Cache StatusSignal objects -- reading .getVelocity() etc. returns the SAME
        // object each time; we store the reference once and call refreshAll() to update all
        // of them in a single CAN transaction instead of one transaction per signal.
        m_velocitySignal      = m_velocityControlMotor.getVelocity();
        m_voltageSignal       = m_velocityControlMotor.getMotorVoltage();
        m_statorCurrentSignal = m_velocityControlMotor.getStatorCurrent();
        m_supplyCurrentSignal = m_velocityControlMotor.getSupplyCurrent();

        // Velocity signal needs to be fresh enough for isAtSetpoint() checks in commands.
        // 50 Hz keeps it at the full robot loop rate; telemetry-only signals can run slower.
        m_velocitySignal.setUpdateFrequency(50);
        m_voltageSignal.setUpdateFrequency(10);
        m_statorCurrentSignal.setUpdateFrequency(10);
        m_supplyCurrentSignal.setUpdateFrequency(10);
    }

    public void setDefaultConstants( VelocityControlConstants constants ){
        m_constants = constants;

        // Configure PIDF settings
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = constants.kP;
        slot0Configs.kI = constants.kI;
        slot0Configs.kD = constants.kD;
        slot0Configs.kV = constants.kV;
        m_velocityControlMotor.getConfigurator().apply(slot0Configs);

        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(constants.kSupplyCurrentLimit)
            .withStatorCurrentLimit(constants.kStatorCurrentLimit);
        m_velocityControlMotor.getConfigurator().apply(limitConfigs);
        

    }

    protected void setInverted(boolean inverted) {
        MotorOutputConfigs motorConfig = new MotorOutputConfigs();
        motorConfig.Inverted = inverted ? 
            InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive;
        m_velocityControlMotor.getConfigurator().apply(motorConfig);
    }

    /**
     * Commands a closed-loop velocity.
     *
     * <p>The public API of every velocity subsystem is RPM. Phoenix works in rotations per second,
     * so the conversion happens here and RPS never escapes this class.
     *
     * @param velocityRPM target rotor velocity in revolutions per minute
     */
    public void setSpeed(double velocityRPM) {
        m_velocityControlMotor.setControl(m_velocityRequest.withVelocity(velocityRPM/60.0)); //RPM to RPS
    }

    /** Rotor velocity in rotations per second, from the cached signal (no extra CAN read). */
    public double getSpeedRPS() {
        return m_velocitySignal.getValueAsDouble();
    }

    /** Rotor velocity in RPM -- the unit this subsystem's public API speaks in. */
    public double getSpeedRPM() {
        return m_velocitySignal.getValueAsDouble() * 60.0;
    }

    public void stopMotor() {
        m_velocityControlMotor.stopMotor();
    }

    /**
     * True when the motor is within toleranceRPM of targetRPM. Uses the cached signal, so it is
     * cheap enough to call from isFinished() every loop.
     */
    public boolean isAtSetpoint(double targetRPM, double toleranceRPM) {
        return Math.abs(getSpeedRPM() - targetRPM) < toleranceRPM;
    }

    /**
     * Slaves this motor to another subsystem's motor.
     *
     * <p>Call this from the follower's constructor, or from RobotContainer right after both
     * subsystems exist. Once following, the follower ignores setSpeed() entirely -- it mirrors the
     * leader's output directly, which is what you want for a two-motor gearbox where the motors
     * are mechanically coupled and must never fight each other.
     *
     * @param leader  the subsystem whose motor this one should mirror
     * @param opposed true when the motors face opposite directions across a shared shaft
     */
    public void followMotor(VelocityControlSystem leader, boolean opposed) {
        m_velocityControlMotor.setControl(new Follower(
            leader.m_velocityControlMotor.getDeviceID(),
            opposed ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
    }

    @Override
    public void periodic() {
        m_periodicCounter++;

        // Only update telemetry every TELEMETRY_PERIOD loops to reduce CAN reads and NT writes
        if (m_periodicCounter % TELEMETRY_PERIOD != 0) return;

        // Refresh all cached signals in ONE CAN transaction instead of 4 separate reads
        BaseStatusSignal.refreshAll(
            m_velocitySignal,
            m_voltageSignal,
            m_statorCurrentSignal,
            m_supplyCurrentSignal
        );

        double rps = m_velocitySignal.getValueAsDouble();
        double rpm = rps * 60.0;

        TelemetryManager.putNumber(m_motorName + " Voltage", m_voltageSignal.getValueAsDouble());
        TelemetryManager.putNumber(m_motorName + " Motor Speed (RPS)", rps);
        TelemetryManager.putNumber(m_motorName + " Motor Speed (RPM)", rpm);
        TelemetryManager.putNumber(m_motorName + " Motor Stator Current", m_statorCurrentSignal.getValueAsDouble());
        TelemetryManager.putNumber(m_motorName + " Motor Supply Current", m_supplyCurrentSignal.getValueAsDouble());
    }

    public static VelocityControlSystem getInstance() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getInstance'");
    }
}
