// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Robot-wide numerical and boolean constants.
 *
 * <p>Nothing functional goes in this class -- no methods that do work, no objects that talk to
 * hardware. Just values. CAN IDs live in {@link RobotMap}; values you want to change from the
 * dashboard while the robot is running live in {@link TunableConstants}.
 *
 * <p><b>Layout of this file, in order:</b>
 * <ol>
 *   <li>Global flags
 *   <li>Operator interface + drivetrain
 *   <li>Vision
 *   <li>The two motor-constant parameter objects
 *   <li>One instance per motor subsystem (the part you edit every season)
 * </ol>
 */
public final class Constants {

  // ===========================================================================
  // GLOBAL FLAGS
  // ===========================================================================

  /**
   * Set true only when collecting SysId / .hoot data. Leave false at competition -- the SignalLogger
   * writes to the roboRIO eMMC flash and can stall the main loop mid-match.
   */
  public static final boolean ENABLE_SIGNAL_LOGGER = false;

  // ===========================================================================
  // OPERATOR INTERFACE
  // ===========================================================================

  public static class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  // ===========================================================================
  // DRIVETRAIN
  // ===========================================================================

  /**
   * Chassis speed caps used by driver-facing commands.
   *
   * <p>These are separate from the values Tuner X generates in {@code TunerConstants} -- those
   * describe what the hardware can physically do, these describe what we let the driver ask for.
   */
  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = 4.0;
  }

  // ===========================================================================
  // LEDS
  // ===========================================================================

  public static final class LEDConstants {
    public static final int kPort = 0;
    public static final int kLength = 120;
  }

  // ===========================================================================
  // VISION
  // ===========================================================================

  /**
   * Limelight pose-estimation tuning. See ARCHITECTURE.md, section "Vision / Pose Estimation", for
   * what each threshold does and the order the rejection checks run in.
   */
  public static final class VisionConstants {
    // Proportional gains for vision-driven alignment commands. Tune these.
    public static final double kAimP = 0.035;   // steering (rotation)
    public static final double kRangeP = 0.1;   // distance (forward/back)

    // Must match the hostname configured in each Limelight web UI.
    public static final String LIMELIGHT_FRONT_NAME = "limelight-front";
    public static final String LIMELIGHT_BACK_NAME = "limelight-back";

    /**
     * MegaTag2 fuses gyro heading into the pose solve. It is off by default: it needs a well-seeded
     * heading to help, and produced poor poses on our 2026 hardware. Re-evaluate each season after
     * a Limelight firmware update -- this is a measurement, not a permanent verdict.
     */
    public static final boolean USE_MEGA_TAG_2 = false;

    // Above this rotation rate the image is motion-blurred enough that we stop trusting it.
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = Units.DegreesPerSecond.of(540);

    // Solver ambiguity, 0 to 1. Lower is a more confident solve; reject anything above this.
    public static final double MAX_AMBIGUITY = 0.3;

    // Limelight 4 resolves tags reliably out to about 4-5 m.
    public static final double MAX_TAG_DISTANCE = 4.5;          // meters, multi-tag
    public static final double MAX_SINGLE_TAG_DISTANCE = 2.0;   // meters, single-tag degrades fast

    // Used only on the conditional heading update (multi-tag + low ambiguity + low omega).
    // About 5 deg is loose enough that heading corrections come in gradually instead of snapping.
    public static final double CONSERVATIVE_HEADING_STD_DEV = Math.toRadians(5.0); // radians

    // Base standard deviations. Vision.java scales these by tag count and distance at runtime.
    public static final double STD_DEVS_POS = 0.7;                 // meters
    public static final double STD_DEVS_HEADING = 9999999;         // radians -- trust gyro, keep high
    public static final double MEGA_TAG1_STD_DEVS_POSITION = 0.5;  // lower trusts vision more
    public static final double MEGA_TAG1_STD_DEVS_HEADING = 9999999;
  }

  // ===========================================================================
  // MOTOR CONSTANT PARAMETER OBJECTS
  //
  // One of these gets built per motor subsystem below, then handed to the base
  // class via setDefaultConstants(). The base class applies it to the TalonFX.
  // ===========================================================================

  /** Tuning values for a subsystem extending {@code VelocityControlSystem}. */
  public static class VelocityControlConstants {
    public final int kCanId;
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kV;
    public final double kSupplyCurrentLimit;
    public final double kStatorCurrentLimit;

    public VelocityControlConstants(int kCanId, double kP, double kI, double kD, double kV,
                                    double kSupplyCurrentLimit, double kStatorCurrentLimit) {
      this.kCanId = kCanId;
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kV = kV;
      this.kSupplyCurrentLimit = kSupplyCurrentLimit;
      this.kStatorCurrentLimit = kStatorCurrentLimit;
    }
  }

  /** Tuning values for a subsystem extending {@code PositionControlSystem}. */
  public static class PositionControlConstants {
    public final int kCanId;
    /** Mechanism travel produced by one motor rotation. Inches for linear mechanisms. */
    public final double kInchesPerRotation;
    /** Motor rotations per one rotation of the mechanism. Drives the angle telemetry readout. */
    public final double kGearRatio;
    public final double kCruiseVelocity;
    public final double kAcceleration;
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kV;
    public final double kSupplyCurrentLimit;
    public final double kStatorCurrentLimit;

    public PositionControlConstants(int kCanId, double kInchesPerRotation, double kGearRatio,
                                    double kCruiseVelocity, double kAcceleration,
                                    double kP, double kI, double kD, double kV,
                                    double kSupplyCurrentLimit, double kStatorCurrentLimit) {
      this.kCanId = kCanId;
      this.kInchesPerRotation = kInchesPerRotation;
      this.kGearRatio = kGearRatio;
      this.kCruiseVelocity = kCruiseVelocity;
      this.kAcceleration = kAcceleration;
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kV = kV;
      this.kSupplyCurrentLimit = kSupplyCurrentLimit;
      this.kStatorCurrentLimit = kStatorCurrentLimit;
    }
  }

  // ===========================================================================
  // PER-SUBSYSTEM CONSTANTS  <- this is the section you rewrite every season
  //
  // The constructors are positional, so the argument ORDER is what binds each
  // number to its field -- not the comment beside it. Miscount by one and kP
  // silently becomes kD, with no compiler error and a mechanism that misbehaves
  // in a way that looks mechanical. Keep the comments aligned and count twice.
  // ===========================================================================

  /** Example velocity subsystem. Delete once you have real mechanisms. */
  public static final VelocityControlConstants ExampleVelocitySubsystemConstants =
      new VelocityControlConstants(
          RobotMap.canIDs.ExampleVelocitySubsystem.MOTOR,
          0.10,   // kP  -- start small, raise until it reaches speed without oscillating
          0.0,    // kI  -- leave at 0 unless you have steady-state error
          0.0,    // kD  -- rarely useful on a velocity loop
          0.12,   // kV  -- feedforward; about 0.12 is typical for a Kraken or Falcon
          40.0,   // kSupplyCurrentLimit (amps)
          60.0    // kStatorCurrentLimit (amps)
      );

  /** Example position subsystem. Delete once you have real mechanisms. */
  public static final PositionControlConstants ExamplePositionSubsystemConstants =
      new PositionControlConstants(
          RobotMap.canIDs.ExamplePositionSubsystem.MOTOR,
          0.8,    // kInchesPerRotation -- measure this, do not guess
          5.0,    // kGearRatio -- write 5.0, not 5/1: integer division truncates silently
          35.0,   // kCruiseVelocity (rotations/sec)
          70.0,   // kAcceleration (rotations/sec^2) -- commonly about 2x cruise
          35.0,   // kP  -- raise until it holds position under load
          0.0,    // kI
          0.0,    // kD  -- add 0.1 to 1.0 if it oscillates around the setpoint
          0.0,    // kV
          40.0,   // kSupplyCurrentLimit (amps)
          60.0    // kStatorCurrentLimit (amps)
      );

  /**
   * Travel limits and tolerances for the example position subsystem.
   *
   * <p>Keep a mechanism's soft limits next to its constants so the clamp inside the subsystem and
   * the setpoints commands ask for can never drift apart.
   */
  public static final class ExamplePositionSubsystemPositions {
    public static final double MIN_POSITION = 0.0;        // inches -- full retract / hard stop
    public static final double MAX_POSITION = 12.0;       // inches -- full extend
    public static final double STOWED = 0.0;              // inches
    public static final double DEPLOYED = 11.5;           // inches
    public static final double POSITION_TOLERANCE = 0.25; // inches
  }

  /**
   * Hard-stop zeroing parameters for the example position subsystem.
   *
   * <p>The stall threshold must sit comfortably BELOW the stator current limit above, or the limit
   * caps current before the detector ever fires and homing silently never completes. That exact bug
   * shipped on the 2026 elevator: a 7.5 A stator limit against a 9.0 A stall threshold.
   */
  public static final class ExamplePositionSubsystemZeroingConstants {
    public static final double RETRACT_VOLTAGE = -1.5;         // volts -- back off the stop first
    public static final double RETRACT_SECONDS = 0.25;
    public static final double ZEROING_VOLTAGE = 2.0;          // volts -- slow, deliberate approach
    public static final double STALL_CURRENT_THRESHOLD = 45.0; // amps -- below the 60 A stator limit
    public static final double STALL_DEBOUNCE_SECONDS = 0.1;   // rejects friction spikes
    public static final double TIMEOUT_SECONDS = 5.0;          // give up rather than cook the motor
    public static final double HARD_STOP_INCHES = 0.0;         // reading to write at the hard stop
  }

  private Constants() {}
}
