// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.utils.TelemetryManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class AddVisionMeasurement extends Command {
  private final Vision vision;
  private final CommandSwerveDrivetrain drivetrain;

  /**
   * NEW: Simple exponential moving average filter for pose smoothing
   */
  private Pose2d m_lastFilteredPose = null;
  private static final double POSE_FILTER_ALPHA = 1.0; // 0 = no smoothing, 1 = no filtering

  // Rate-limiting counter for telemetry
  private int m_telemetryCounter = 0;
  private static final int TELEMETRY_PERIOD = 5; // Every 5 loops (100ms)

  /**
   * Adds vision measurements to the drivetrain's pose estimator.
   * This command does NOT require the drivetrain, allowing it to run
   * in parallel with driving commands.
   * 
   * ENHANCED with:
   * - Dynamic standard deviations based on tag count and distance
   * - Optional pose filtering
   * 
   * @param vision     The vision subsystem
   * @param drivetrain The drivetrain subsystem
   */
  public AddVisionMeasurement(Vision vision, CommandSwerveDrivetrain drivetrain) {
    this.vision = vision;
    this.drivetrain = drivetrain;
    
    // Do not addRequirement(drivetrain) -> We want this to run ALONGSIDE driving
    addRequirements(vision);
  }

  /**
   * NEW: Calculate dynamic standard deviations based on measurement quality.
   * More tags and closer tags = lower standard deviation = more trust.
   *
   * Also applies conservative heading correction when all quality criteria are met:
   *  - >=2 tags visible
   *  - avg tag distance < 2.5m
   *  - max ambiguity < 0.2 (stricter than general threshold)
   *  - robot angular velocity < 10 deg/s (nearly stationary in rotation)
   *
   * @param estimate    The pose estimate
   * @param gyroRateRad Current robot angular velocity in rad/s
   * @return Standard deviation matrix [x, y, theta]
   */
  private Matrix<N3, N1> calculateDynamicStdDevs(PoseEstimate estimate, double gyroRateRad) {
    // Base standard deviations (from constants)
    double xyStdDev = VisionConstants.MEGA_TAG1_STD_DEVS_POSITION;

    // Tag count scaling: 1 tag = 1.0x, 2 tags = 0.5x, 3+ tags = 0.33x
    double tagCountFactor;
    if (estimate.tagCount >= 3) {
      tagCountFactor = 0.33;
    } else if (estimate.tagCount == 2) {
      tagCountFactor = 0.5;
    } else {
      tagCountFactor = 1.0;
    }

    // Distance scaling: closer tags get more trust (smaller factor -> smaller stddev -> more trust).
    // At 0.5m -> factor=0.5 (trusted more), at 1.0m -> factor=1.0 (base), at 2.0m+ -> factor<=2.0 (less trusted).
    double distanceFactor = Math.max(0.1, Math.min(2.0, estimate.avgTagDist));

    // Combined scaling, clamped to reasonable bounds
    double scaledXYStdDev = Math.max(0.1, Math.min(2.0, xyStdDev * tagCountFactor * distanceFactor));

    // --- Conservative heading correction ---
    // Only allow vision to nudge heading when ALL quality criteria are met.
    // Effect is gradual (~5 deg stddev) to correct gyro drift without jitter.
    double maxAmbiguity = 0.0;
    if (estimate.rawFiducials != null) {
      for (var fid : estimate.rawFiducials) {
        maxAmbiguity = Math.max(maxAmbiguity, fid.ambiguity);
      }
    }
    boolean headingQualityMet =
        estimate.tagCount >= 2
        && estimate.avgTagDist < 2.5
        && maxAmbiguity < 0.2
        && Math.abs(gyroRateRad) < Math.toRadians(10.0);

    double thetaStdDev = headingQualityMet
        ? VisionConstants.CONSERVATIVE_HEADING_STD_DEV
        : VisionConstants.STD_DEVS_HEADING; // 9999999 (ignore heading)

    return VecBuilder.fill(scaledXYStdDev, scaledXYStdDev, thetaStdDev);
  }

  /**
   * NEW: Optional exponential moving average filter for pose smoothing.
   * This can help reduce jitter from frame-to-frame noise.
   * Set POSE_FILTER_ALPHA = 1.0 to disable filtering.
   * 
   * @param newPose The new pose from vision
   * @return Filtered pose
   */
  private Pose2d filterPose(Pose2d newPose) {
    if (m_lastFilteredPose == null) {
      m_lastFilteredPose = newPose;
      return newPose;
    }
    
    // Simple exponential moving average on X and Y
    // Note: We're NOT filtering rotation since we ignore vision heading anyway
    double filteredX = POSE_FILTER_ALPHA * newPose.getX() + (1 - POSE_FILTER_ALPHA) * m_lastFilteredPose.getX();
    double filteredY = POSE_FILTER_ALPHA * newPose.getY() + (1 - POSE_FILTER_ALPHA) * m_lastFilteredPose.getY();
    
    Pose2d filteredPose = new Pose2d(filteredX, filteredY, newPose.getRotation());
    m_lastFilteredPose = filteredPose;
    
    return filteredPose;
  }

  @Override
  public void execute() {
    m_telemetryCounter++;
    boolean doTelemetry = (m_telemetryCounter % TELEMETRY_PERIOD == 0);

    // Use cached state -- getState() acquires an internal lock; getCachedState() reuses the
    // snapshot already taken once in CommandSwerveDrivetrain.periodic() this loop.
    var driveState = drivetrain.getCachedState();

    // 1. Get current rotation rate from Drivetrain (needed for Vision rejection logic)
    double gyroRateRadPerSec = driveState.Speeds.omegaRadiansPerSecond;
    AngularVelocity gyroRate = Units.RadiansPerSecond.of(gyroRateRadPerSec);

    // 2. Ask Vision subsystem for the best available pose estimate
    Optional<PoseEstimate> bestEstimate = vision.determinePoseEstimate(gyroRate);

    // 3. If a valid estimate exists, send it to the Drivetrain pose estimator
    if (bestEstimate.isPresent()) {
      PoseEstimate estimate = bestEstimate.get();
      
      // Apply optional pose filtering (set POSE_FILTER_ALPHA = 1.0 to disable)
      Pose2d poseToUse = estimate.pose;
      if (POSE_FILTER_ALPHA < 1.0) {
        poseToUse = filterPose(estimate.pose);
      }
      
      // Calculate dynamic standard deviations (pass gyro rate for heading correction logic)
      Matrix<N3, N1> dynamicStdDevs = calculateDynamicStdDevs(estimate, gyroRateRadPerSec);
      
      // Add the measurement to the drivetrain
      drivetrain.addVisionMeasurement(
          poseToUse,
          estimate.timestampSeconds,
          dynamicStdDevs);
      
      if (doTelemetry) {
        TelemetryManager.putNumber("Vision/Accepted Tag Count", estimate.tagCount);
        TelemetryManager.putNumber("Vision/Accepted Avg Dist", estimate.avgTagDist);
        TelemetryManager.putBoolean("Vision/Measurement Added", true);
      }
    } else {
      if (doTelemetry) {
        TelemetryManager.putBoolean("Vision/Measurement Added", false);
      }
    }

    // Rate-limited debug telemetry
      if (doTelemetry) {
        TelemetryManager.putNumber("Vision/Gyro Rate (rad_s)", gyroRateRadPerSec);
      
        if (bestEstimate.isPresent()) {
          double visionHeading = bestEstimate.get().pose.getRotation().getDegrees();
          double gyroHeading = driveState.Pose.getRotation().getDegrees();
          double headingDifference = Math.abs(visionHeading - gyroHeading);
          TelemetryManager.putNumber("Vision/Heading Difference (deg)", headingDifference);
        }
      }
  }

  @Override
  public boolean isFinished() {
    // Runs forever (until interrupted)
    return false;
  }
  
  @Override
  public boolean runsWhenDisabled() {
    // Allow vision updates even when disabled (useful for initializing pose before auto)
    return true;
  }
}
