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
  //
  // Both are built with a BUILDER rather than a constructor. A constructor taking
  // eleven doubles binds each value by argument POSITION, so miscounting by one
  // turns kP into kD with no compiler error -- the mechanism then misbehaves in a
  // way that looks mechanical, and the first hour goes into checking the gearbox.
  // With a builder the method name binds the value, so order cannot matter.
  //
  // The trade is that you can now forget a call instead of mis-ordering one, so
  // build() verifies that every group a zero would silently break was supplied.
  // These objects are created in a static initializer, which means a missing
  // group is a hard failure at robot boot, with a message naming the missing
  // call -- loud, immediate, and identical every single run.
  // ===========================================================================

  /**
   * Tuning values for a subsystem extending {@code VelocityControlSystem}.
   *
   * <p>Build one with {@link #forMotor(int)}:
   *
   * <pre>{@code
   * VelocityControlConstants.forMotor(RobotMap.canIDs.Flywheel.MOTOR)
   *     .withPID(0.10, 0.0, 0.0, 0.12)
   *     .withCurrentLimits(40.0, 60.0)
   *     .build();
   * }</pre>
   */
  public static final class VelocityControlConstants {
    public final int kCanId;
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kV;
    public final double kSupplyCurrentLimit;
    public final double kStatorCurrentLimit;

    private VelocityControlConstants(Builder b) {
      this.kCanId = b.canId;
      this.kP = b.kP;
      this.kI = b.kI;
      this.kD = b.kD;
      this.kV = b.kV;
      this.kSupplyCurrentLimit = b.supplyCurrentLimit;
      this.kStatorCurrentLimit = b.statorCurrentLimit;
    }

    /** Starts a builder for the motor at {@code canId}. Take the ID from {@link RobotMap}. */
    public static Builder forMotor(int canId) {
      return new Builder(canId);
    }

    /** Fluent builder. Each {@code with*} call returns itself; finish with {@code build()}. */
    public static final class Builder {
      private final int canId;
      private double kP;
      private double kI;
      private double kD;
      private double kV;
      private double supplyCurrentLimit;
      private double statorCurrentLimit;
      private boolean pidSet;
      private boolean currentLimitsSet;

      private Builder(int canId) {
        this.canId = canId;
      }

      /**
       * Closed-loop velocity gains. Required.
       *
       * @param kP proportional -- start small, raise until it holds speed without oscillating
       * @param kI integral -- leave at 0 unless you have steady-state error
       * @param kD derivative -- rarely useful on a velocity loop
       * @param kV velocity feedforward -- about 0.12 is typical for a Kraken or Falcon
       */
      public Builder withPID(double kP, double kI, double kD, double kV) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.pidSet = true;
        return this;
      }

      /**
       * Current limits in amps. Required.
       *
       * @param supplyAmps drawn from the battery
       * @param statorAmps delivered to the windings -- this is the one that caps torque
       */
      public Builder withCurrentLimits(double supplyAmps, double statorAmps) {
        this.supplyCurrentLimit = supplyAmps;
        this.statorCurrentLimit = statorAmps;
        this.currentLimitsSet = true;
        return this;
      }

      /** Validates that nothing required was left out, then produces the constants object. */
      public VelocityControlConstants build() {
        require(pidSet, canId, "withPID(kP, kI, kD, kV)");
        require(currentLimitsSet, canId, "withCurrentLimits(supplyAmps, statorAmps)");
        return new VelocityControlConstants(this);
      }
    }
  }

  /**
   * Tuning values for a subsystem extending {@code PositionControlSystem}.
   *
   * <p>Build one with {@link #forMotor(int)}:
   *
   * <pre>{@code
   * PositionControlConstants.forMotor(RobotMap.canIDs.Elevator.MOTOR)
   *     .withInchesPerRotation(0.8)
   *     .withGearRatio(5.0)
   *     .withMotionMagic(35.0, 70.0)
   *     .withPID(35.0, 0.0, 0.0, 0.0)
   *     .withCurrentLimits(40.0, 60.0)
   *     .build();
   * }</pre>
   */
  public static final class PositionControlConstants {
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

    private PositionControlConstants(Builder b) {
      this.kCanId = b.canId;
      this.kInchesPerRotation = b.inchesPerRotation;
      this.kGearRatio = b.gearRatio;
      this.kCruiseVelocity = b.cruiseVelocity;
      this.kAcceleration = b.acceleration;
      this.kP = b.kP;
      this.kI = b.kI;
      this.kD = b.kD;
      this.kV = b.kV;
      this.kSupplyCurrentLimit = b.supplyCurrentLimit;
      this.kStatorCurrentLimit = b.statorCurrentLimit;
    }

    /** Starts a builder for the motor at {@code canId}. Take the ID from {@link RobotMap}. */
    public static Builder forMotor(int canId) {
      return new Builder(canId);
    }

    /** Fluent builder. Each {@code with*} call returns itself; finish with {@code build()}. */
    public static final class Builder {
      private final int canId;
      private double inchesPerRotation;
      private double gearRatio = 1.0;
      private double cruiseVelocity;
      private double acceleration;
      private double kP;
      private double kI;
      private double kD;
      private double kV;
      private double supplyCurrentLimit;
      private double statorCurrentLimit;
      private boolean inchesPerRotationSet;
      private boolean motionMagicSet;
      private boolean pidSet;
      private boolean currentLimitsSet;

      private Builder(int canId) {
        this.canId = canId;
      }

      /**
       * Mechanism travel per motor rotation, in inches. Required -- measure it, do not guess.
       *
       * <p>Every inches/rotations conversion in the base class divides by this, so a value of zero
       * would command a position of infinity. That is why it has no default.
       */
      public Builder withInchesPerRotation(double inchesPerRotation) {
        this.inchesPerRotation = inchesPerRotation;
        this.inchesPerRotationSet = true;
        return this;
      }

      /**
       * Motor rotations per one mechanism rotation. Optional, defaults to 1.0.
       *
       * <p>Used only for the angle telemetry readout, so leaving it out costs you a dashboard
       * number and nothing else. Set it on rotary mechanisms; skip it on linear ones.
       *
       * <p>Write {@code 5.0}, not {@code 5/1} -- integer division truncates silently.
       */
      public Builder withGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
        return this;
      }

      /**
       * Motion Magic trapezoidal profile limits. Required.
       *
       * @param cruiseVelocity rotations per second at the flat top of the profile
       * @param acceleration rotations per second squared on the ramps; commonly about 2x cruise
       */
      public Builder withMotionMagic(double cruiseVelocity, double acceleration) {
        this.cruiseVelocity = cruiseVelocity;
        this.acceleration = acceleration;
        this.motionMagicSet = true;
        return this;
      }

      /**
       * Closed-loop position gains. Required.
       *
       * @param kP proportional -- raise until it holds position under load
       * @param kI integral -- usually 0
       * @param kD derivative -- add 0.1 to 1.0 if it oscillates around the setpoint
       * @param kV velocity feedforward
       */
      public Builder withPID(double kP, double kI, double kD, double kV) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.pidSet = true;
        return this;
      }

      /**
       * Current limits in amps. Required.
       *
       * <p>Keep the stator limit comfortably above any stall threshold used for hard-stop zeroing,
       * or the limit caps current before the stall detector fires and homing never completes.
       *
       * @param supplyAmps drawn from the battery
       * @param statorAmps delivered to the windings -- this is the one that caps torque
       */
      public Builder withCurrentLimits(double supplyAmps, double statorAmps) {
        this.supplyCurrentLimit = supplyAmps;
        this.statorCurrentLimit = statorAmps;
        this.currentLimitsSet = true;
        return this;
      }

      /** Validates that nothing required was left out, then produces the constants object. */
      public PositionControlConstants build() {
        require(inchesPerRotationSet, canId, "withInchesPerRotation(inches)");
        require(motionMagicSet, canId, "withMotionMagic(cruiseVelocity, acceleration)");
        require(pidSet, canId, "withPID(kP, kI, kD, kV)");
        require(currentLimitsSet, canId, "withCurrentLimits(supplyAmps, statorAmps)");
        return new PositionControlConstants(this);
      }
    }
  }

  /**
   * Shared by both builders. Fails the build with a message naming the call that was missed.
   *
   * <p>Thrown from a static initializer, so the JVM wraps it in an {@code ExceptionInInitializerError}
   * at boot. The message still prints, and the failure is the same every run -- which is the point.
   * Silently defaulting a required gain to zero produces a mechanism that simply does not move, and
   * that is much harder to trace back to this file.
   */
  private static void require(boolean wasSet, int canId, String missingCall) {
    if (!wasSet) {
      throw new IllegalStateException(
          "Motor constants for CAN ID " + canId + " are incomplete: " + missingCall
              + " was never called before build(). Leaving it out would set those values to zero,"
              + " which does not fail loudly at runtime.");
    }
  }

  // ===========================================================================
  // PER-SUBSYSTEM CONSTANTS  <- this is the section you rewrite every season
  //
  // Copy one of the blocks below, rename it, point forMotor() at your CAN ID,
  // and change the numbers. The builder call order does not matter and the
  // compiler checks each value against the parameter it is named for, so the
  // only thing to be careful about is the numbers themselves.
  // ===========================================================================

  /** Example velocity subsystem. Delete once you have real mechanisms. */
  public static final VelocityControlConstants ExampleVelocitySubsystemConstants =
      VelocityControlConstants.forMotor(RobotMap.canIDs.ExampleVelocitySubsystem.MOTOR)
          .withPID(0.10, 0.0, 0.0, 0.12)  // kP, kI, kD, kV
          .withCurrentLimits(40.0, 60.0)  // supply amps, stator amps
          .build();

  /** Example position subsystem. Delete once you have real mechanisms. */
  public static final PositionControlConstants ExamplePositionSubsystemConstants =
      PositionControlConstants.forMotor(RobotMap.canIDs.ExamplePositionSubsystem.MOTOR)
          .withInchesPerRotation(0.8)     // measure this, do not guess
          .withGearRatio(5.0)             // telemetry only; omit on a linear mechanism
          .withMotionMagic(35.0, 70.0)    // cruise rot/s, accel rot/s^2
          .withPID(35.0, 0.0, 0.0, 0.0)   // kP, kI, kD, kV
          .withCurrentLimits(40.0, 60.0)  // supply amps, stator amps
          .build();

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
