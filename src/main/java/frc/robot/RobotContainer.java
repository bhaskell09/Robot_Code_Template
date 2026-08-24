// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.ExampleNamedCommand;
import frc.robot.commands.PositionBasedCommands.SetExamplePositionSubsystem;
import frc.robot.commands.PositionBasedCommands.ZeroExamplePositionSubsystem;
import frc.robot.commands.VelocityBasedCommands.RunExampleVelocitySubsystem;
import frc.robot.constants.Constants.ExamplePositionSubsystemPositions;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.constants.RobotMap;
import frc.robot.constants.TunableConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.position.ExamplePositionSubsystem;
import frc.robot.subsystems.velocity.ExampleVelocitySubsystem;
import frc.robot.utils.TelemetryManager;

/**
 * Where the robot is assembled: subsystems are created, buttons are bound to commands, and
 * autonomous is wired up.
 *
 * <p>This class holds no control logic of its own. If you find yourself writing math here, it
 * belongs in a subsystem or a command -- keeping this file a wiring diagram is what makes it
 * readable when six people are editing it in the same week.
 *
 * <p>See ARCHITECTURE.md for how the pieces fit together.
 */
public class RobotContainer {

    // ===========================================================================
    // SUBSYSTEMS
    // ===========================================================================

    private final Vision m_vision = new Vision();

    // Example mechanisms. Delete these and add your season's subsystems here.
    private final ExampleVelocitySubsystem m_exampleVelocity = new ExampleVelocitySubsystem();
    private final ExamplePositionSubsystem m_examplePosition = new ExamplePositionSubsystem();

    // ===========================================================================
    // OPERATOR INTERFACE
    // ===========================================================================

    private final CommandXboxController m_driverController =
        new CommandXboxController(OIConstants.kDriverControllerPort);

    private final CommandXboxController m_operatorController =
        new CommandXboxController(OIConstants.kOperatorControllerPort);

    // ===========================================================================
    // DRIVETRAIN
    // ===========================================================================

    /** Top translational speed we let the driver request. */
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    /** Top rotational speed: 3/4 rotation per second. */
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Slew rate limiting smooths acceleration at the cost of a less immediate feel. Off by default;
    // to enable, uncomment these and swap in the limiter lines inside the default drive command.
    //   private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(2.5);  // m/s^2
    //   private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(2.5);
    //   private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(4.5);     // rad/s^2
    // Also add: import edu.wpi.first.math.filter.SlewRateLimiter;

    /* Swerve requests, built once and reused -- allocating these per loop would churn the GC. */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05)
            .withRotationalDeadband(MaxAngularRate * 0.05)   // 5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // ===========================================================================
    // MISC
    // ===========================================================================

    private final SendableChooser<Command> autoChooser;

    private final CANBus canivore = new CANBus(RobotMap.canBusNames.CANIVORE);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // 1. Telemetry. Disabled reduces NT and CAN load; turn it on while debugging.
        TelemetryManager.setEnabled(false);  // Set to false for competition

        // 2. Publish tunable defaults so the keys exist in the dashboard tree at boot.
        TunableConstants.initializeDefaults();

        // 3. Vision runs continuously in the background, feeding the pose estimator.
        m_vision.setDefaultCommand(new AddVisionMeasurement(m_vision, drivetrain));

        // 4. PathPlanner NamedCommands.
        //
        //    Registration MUST happen before configurePathPlanner() and buildAutoChooser() below.
        //    PathPlanner resolves these names when the autos load, so anything registered after
        //    that point is silently ignored -- the event marker does nothing, and there is no error
        //    at deploy and no warning at runtime. Names are case-sensitive and must match the
        //    .auto file exactly.
        NamedCommands.registerCommand("ExampleNamedCommand",
            new ExampleNamedCommand(m_exampleVelocity, m_examplePosition));

        // 5. Autonomous setup, in this order.
        drivetrain.seedFieldCentric();
        drivetrain.configurePathPlanner();
        autoChooser = AutoBuilder.buildAutoChooser();

        // Posted directly rather than through TelemetryManager: the chooser must always appear,
        // even when telemetry is disabled for competition.
        SmartDashboard.putData("Auto Mode", autoChooser);

        // 6. Button bindings.
        configureBindings();
    }

    private void configureBindings() {

        // ========================================
        // DRIVER CONTROLLER BINDINGS
        // ========================================

        // Default drive command (field centric).
        // WPILib convention: +X is forward, +Y is to the left. The negations convert from the
        // controller's convention, where pushing the stick forward reads negative.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)
                // Slew-limited alternative, replacing the three lines above:
                //   drive.withVelocityX(m_xSpeedLimiter.calculate(-m_driverController.getLeftY() * MaxSpeed))
                //       .withVelocityY(m_ySpeedLimiter.calculate(-m_driverController.getLeftX() * MaxSpeed))
                //       .withRotationalRate(m_rotLimiter.calculate(-m_driverController.getRightX() * MaxAngularRate))
            )
        );

        // X-lock the wheels -- holds position against a push.
        m_driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));

        // Point all modules where the left stick is aimed. Useful for diagnosing steer offsets.
        m_driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(
                new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        ));

        // Re-zero field-centric heading to whichever way the robot is currently facing.
        m_driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Snap the pose estimate to what the cameras see. Handy after a collision.
        m_driverController.povUp().onTrue(drivetrain.resetToVisionPoseCommand(m_vision));

        // SysId characterization routines (Back/Start + face buttons).
        // Each routine must be run exactly once per log, and only with the robot on blocks or on a
        // long clear stretch of carpet. Set Constants.ENABLE_SIGNAL_LOGGER = true first, and back
        // to false before any competition deploy.
        m_driverController.back().and(m_driverController.x())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.back().and(m_driverController.y())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.start().and(m_driverController.x())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

        // ========================================
        // OPERATOR CONTROLLER BINDINGS
        //
        // Mechanism controls live here, so the driver's controller stays dedicated to driving.
        // The bindings below demonstrate the two subsystem patterns -- replace them with yours.
        // ========================================

        // Velocity: runs while held, stops on release.
        m_operatorController.a().whileTrue(
            new RunExampleVelocitySubsystem(m_exampleVelocity, 2000.0, true));

        // Position: goes to a setpoint and holds it there after the command ends.
        m_operatorController.b().onTrue(
            new SetExamplePositionSubsystem(
                m_examplePosition, ExamplePositionSubsystemPositions.DEPLOYED));

        m_operatorController.x().onTrue(
            new SetExamplePositionSubsystem(
                m_examplePosition, ExamplePositionSubsystemPositions.STOWED));

        // Homing. Give this its own button so a mechanism that loses its zero mid-match can be
        // recovered without a reboot.
        m_operatorController.y().onTrue(new ZeroExamplePositionSubsystem(m_examplePosition));

        // ========================================
        // RUMBLE
        //
        // Haptics are how a driver learns something without looking away from the field.
        // Bind them to states worth interrupting for: a game piece acquired, a shot ready,
        // a mechanism refusing to move. See ARCHITECTURE.md, "Haptic Feedback".
        // ========================================

        // ========================================
        // ROBOT MODE TRIGGERS
        // ========================================

        // Hold the drivetrain in its configured neutral mode while disabled. ignoringDisable(true)
        // is what makes this run at all -- the scheduler skips commands while disabled otherwise.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Home mechanisms at the start of each mode. A mechanism that homed in auto has been moving
        // since, so re-establishing the reference at the teleop boundary costs a second and removes
        // a whole class of "it worked in auto" problems.
        RobotModeTriggers.autonomous().onTrue(new ZeroExamplePositionSubsystem(m_examplePosition));
        RobotModeTriggers.teleop().onTrue(new ZeroExamplePositionSubsystem(m_examplePosition));

        // ========================================
        // TELEMETRY
        // ========================================

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Command auto = autoChooser.getSelected();

        // Seed the pose from vision before the path starts, so the first path segment begins from
        // where the robot actually is rather than from wherever the estimator drifted to while
        // sitting on the field. The timeout matters: if no tag is visible, run the auto anyway
        // rather than burning autonomous seconds waiting for one.
        //
        // .asProxy() tells the scheduler to run the selected auto without this composition taking
        // ownership of its requirements -- otherwise the composition would hold every subsystem the
        // auto touches for its whole duration.
        return drivetrain.resetToVisionPoseCommand(m_vision)
            .withTimeout(0.5)
            .andThen(auto.asProxy());
    }

    // ===========================================================================
    // PERIODIC
    //
    // Called from Robot.robotPeriodic(). This is for work that is not tied to any
    // one subsystem -- a subsystem's own periodic() is the right place for its own
    // telemetry.
    // ===========================================================================

    private int m_periodicCounter = 0;

    public void periodic() {
        m_periodicCounter++;

        // Every 10 loops (200 ms). Health telemetry changes slowly, and NT writes are not free.
        if (m_periodicCounter % 10 == 0) {
            updateTelemetry();
        }
    }

    /**
     * Robot health telemetry: power, brownout state, and both CAN buses.
     *
     * <p>Worth keeping even when you strip everything else. Rising CAN bus utilization or a
     * climbing error count is the earliest visible sign of a failing wire or a dying device, and it
     * shows up here well before the mechanism it feeds starts behaving strangely.
     */
    private void updateTelemetry() {
        TelemetryManager.putNumber("Telemetry/BatteryVoltage", RobotController.getBatteryVoltage());
        TelemetryManager.putNumber("Telemetry/BrownoutVoltage", RobotController.getBrownoutVoltage());
        TelemetryManager.putBoolean("Telemetry/isBrownedOut", RobotController.isBrownedOut());
        TelemetryManager.putNumber("Telemetry/getCurrent3V3", RobotController.getCurrent3V3());
        TelemetryManager.putNumber("Telemetry/getCurrent5V", RobotController.getCurrent5V());
        TelemetryManager.putNumber("Telemetry/getCurrent6V", RobotController.getCurrent6V());

        // roboRIO CAN bus
        CANStatus status = RobotController.getCANStatus();
        TelemetryManager.putNumber("Telemetry/CAN Bus Utilization (%)", status.percentBusUtilization * 100);
        TelemetryManager.putNumber("Telemetry/CAN Bus Off Count", status.busOffCount);
        TelemetryManager.putNumber("Telemetry/CAN TX Full Count", status.txFullCount);
        TelemetryManager.putNumber("Telemetry/CAN Receive Error Count", status.receiveErrorCount);
        TelemetryManager.putNumber("Telemetry/CAN Transmit Error Count", status.transmitErrorCount);

        // CANivore (drivetrain) bus
        CANBus.CANBusStatus canivoreStatus = canivore.getStatus();
        TelemetryManager.putNumber("Telemetry/CANivore Bus Utilization (%)", canivoreStatus.BusUtilization * 100);
        TelemetryManager.putNumber("Telemetry/CANivore Bus Off Count", canivoreStatus.BusOffCount);
        TelemetryManager.putNumber("Telemetry/CANivore TX Full Count", canivoreStatus.TxFullCount);
        TelemetryManager.putNumber("Telemetry/CANivore Receive Error Count", canivoreStatus.REC);
        TelemetryManager.putNumber("Telemetry/CANivore Transmit Error Count", canivoreStatus.TEC);
    }
}
