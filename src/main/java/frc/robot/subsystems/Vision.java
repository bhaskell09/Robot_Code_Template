// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.utils.TelemetryManager;

@Logged
public class Vision extends SubsystemBase {
  PoseEstimate lastEstimateFront = new PoseEstimate();
  PoseEstimate lastEstimateBack = new PoseEstimate();

  private final Field2d m_visionField = new Field2d();

  // Not logged, as they turn to false immediately after being read
  @NotLogged
  boolean newFrontEstimate = false;
  @NotLogged
  boolean newBackEstimate = false;

  Pose2d frontPose = new Pose2d();
  Pose2d backPose = new Pose2d();

  private boolean useMegaTag2 = Constants.VisionConstants.USE_MEGA_TAG_2;

  // ============ NEW: Limelight NT entry cache for timestamp pre-check ============
  // These are the same entries LimelightHelpers reads internally -- we peek at their
  // NT server timestamp BEFORE calling the full (expensive) getBotPoseEstimate parse.
  // Because getLimelightDoubleArrayEntry() is itself cached, grabbing the entry is free.
  @NotLogged
  private static final String MEGATAG2_ENTRY = "botpose_orb_wpiblue";
  @NotLogged
  private static final String MEGATAG1_ENTRY = "botpose_wpiblue";

  // Last NT server timestamps seen for each camera (microseconds)
  @NotLogged
  private long m_lastFrontNtTimestamp = -1;
  @NotLogged
  private long m_lastBackNtTimestamp = -1;

  // Minimum wall-clock interval between Jackson JSON parses per camera.
  // Even if the NT timestamp changed (camera at 90fps), we skip the parse if
  // we parsed less than VISION_MIN_PARSE_INTERVAL seconds ago. This caps CPU
  // usage on the single-core roboRIO 1 without losing meaningful pose data.
  @NotLogged
  private double m_lastFrontParseTime = 0.0;
  @NotLogged
  private double m_lastBackParseTime = 0.0;
  private static final double VISION_MIN_PARSE_INTERVAL = 0.015; // ~66 Hz max parse rate

  // Cached PoseEstimate from the last successful parse -- reused when NT hasn't updated
  @NotLogged
  private PoseEstimate m_cachedFrontEstimate = null;
  @NotLogged
  private PoseEstimate m_cachedBackEstimate = null;

  // Rate-limiting counter for telemetry
  @NotLogged
  private int m_telemetryCounter = 0;
  private static final int VISION_TELEMETRY_PERIOD = 5; // Every 5 loops (100ms)

  // ============ NEW: Camera Selection Stability ============
  private enum PreferredCamera {
    NONE,
    FRONT,
    BACK
  }
  
  @NotLogged  // FIX: Don't try to log this - it causes compilation errors with Epilogue
  private PreferredCamera m_preferredCamera = PreferredCamera.NONE;
  
  @NotLogged  // FIX: Don't log timing variables
  private double m_lastCameraSwitchTime = 0.0;
  
  private static final double CAMERA_SWITCH_COOLDOWN = 0.5; // Don't switch cameras more than once every 0.5s
  private static final double DISTANCE_ADVANTAGE_THRESHOLD = 0.3; // Camera must be 0.3m closer to switch

  // ============ NEW: Timestamp Validation ============
  @NotLogged  // FIX: Don't log internal timestamp tracking
  private double m_lastAcceptedTimestamp = 0.0;
  
  private static final double MAX_TIMESTAMP_JUMP = 0.5; // Reject if timestamp jumps >0.5s into future
  
  public Vision() {
    TelemetryManager.putData("Vision Field", m_visionField);
  }

  public PoseEstimate[] getLastPoseEstimates() {
    return new PoseEstimate[] { lastEstimateFront, lastEstimateBack };
  }

  public void setMegaTag2(boolean useMegaTag2) {
    this.useMegaTag2 = useMegaTag2;
  }

  /**
   * Determines if a given pose estimate should be rejected.
   * ENHANCED with additional validation checks.
   * 
   * @param poseEstimate The pose estimate to check
   * @param gyroRate     The current rate of rotation observed by our gyro.
   * @param areaThreshold The minimum tag area required to accept a single-tag estimate.
   * 
   * @return True if the estimate should be rejected
   */
  public boolean rejectUpdate(PoseEstimate poseEstimate, AngularVelocity gyroRate) {
    // Angular velocity is too high to have accurate vision
    if (gyroRate.compareTo(Constants.VisionConstants.MAX_ANGULAR_VELOCITY) > 0) {
      return true;
    }

    // No tags :<
    if (poseEstimate.tagCount == 0) {
      return true;
    }
    
    // NEW: Reject if pose is null
    if (poseEstimate.pose == null) {
      return true;
    }
    
    // NEW: Reject if timestamp is invalid (0 or stale)
    if (poseEstimate.timestampSeconds <= 0) {
      return true;
    }

    double currentTime = Timer.getFPGATimestamp();

    // NEW: Reject if timestamp jumps too far into the future
    if (poseEstimate.timestampSeconds > currentTime + MAX_TIMESTAMP_JUMP) {
      return true;
    }

    // Reject stale data -- poses older than 200ms are too outdated to inject into the estimator
    if (poseEstimate.timestampSeconds < currentTime - 0.2) {
      return true;
    }
    
    // NEW: Reject if this is older than our last accepted measurement
    if (poseEstimate.timestampSeconds <= m_lastAcceptedTimestamp) {
      return true;
    }
    
    // Check if we have raw fiducial data
    if (poseEstimate.rawFiducials == null || poseEstimate.rawFiducials.length == 0) {
      return true;
    }
    
    // Check ambiguity of the primary tag
    if (poseEstimate.rawFiducials[0].ambiguity > Constants.VisionConstants.MAX_AMBIGUITY) {
      return true;
    }

    // Check distance of the primary tag
    if (poseEstimate.rawFiducials[0].distToCamera > Constants.VisionConstants.MAX_TAG_DISTANCE) {
      return true;
    }

    // Multi-tag estimates are always accepted (distance/ambiguity already checked above)
    if (poseEstimate.tagCount >= 2) {
      return false;
    }

    // Single-tag: accept only if the tag is close enough that the estimate is reliable.
    // Beyond MAX_SINGLE_TAG_DISTANCE, single-tag accuracy degrades unacceptably.
    return poseEstimate.avgTagDist > Constants.VisionConstants.MAX_SINGLE_TAG_DISTANCE;
  }

  /**
   * NEW: Determines which camera to use with hysteresis to prevent rapid switching.
   * 
   * @param frontValid Is the front camera estimate valid?
   * @param backValid Is the back camera estimate valid?
   * @return The preferred camera to use
   */
  private PreferredCamera selectPreferredCamera(boolean frontValid, boolean backValid) {
    double currentTime = Timer.getFPGATimestamp();
    double timeSinceLastSwitch = currentTime - m_lastCameraSwitchTime;
    
    // If only one camera is valid, use that one
    if (frontValid && !backValid) {
      if (m_preferredCamera != PreferredCamera.FRONT) {
        m_preferredCamera = PreferredCamera.FRONT;
        m_lastCameraSwitchTime = currentTime;
      }
      return PreferredCamera.FRONT;
    }
    
    if (!frontValid && backValid) {
      if (m_preferredCamera != PreferredCamera.BACK) {
        m_preferredCamera = PreferredCamera.BACK;
        m_lastCameraSwitchTime = currentTime;
      }
      return PreferredCamera.BACK;
    }
    
    // If neither valid, return NONE
    if (!frontValid && !backValid) {
      return PreferredCamera.NONE;
    }
    
    // Both cameras valid - apply hysteresis
    double frontDist = lastEstimateFront.avgTagDist;
    double backDist = lastEstimateBack.avgTagDist;
    
    // If we recently switched, stick with current camera unless the other is MUCH better
    if (timeSinceLastSwitch < CAMERA_SWITCH_COOLDOWN) {
      switch (m_preferredCamera) {
        case FRONT:
          // Only switch to back if it's significantly closer
          if (backDist < frontDist - DISTANCE_ADVANTAGE_THRESHOLD) {
            m_preferredCamera = PreferredCamera.BACK;
            m_lastCameraSwitchTime = currentTime;
          }
          break;
        case BACK:
          // Only switch to front if it's significantly closer
          if (frontDist < backDist - DISTANCE_ADVANTAGE_THRESHOLD) {
            m_preferredCamera = PreferredCamera.FRONT;
            m_lastCameraSwitchTime = currentTime;
          }
          break;
        case NONE:
          // First time - choose closer camera
          m_preferredCamera = (frontDist < backDist) ? PreferredCamera.FRONT : PreferredCamera.BACK;
          m_lastCameraSwitchTime = currentTime;
          break;
      }
    } else {
      // Cooldown expired - can freely switch to closer camera
      PreferredCamera newPreferred = (frontDist < backDist) ? PreferredCamera.FRONT : PreferredCamera.BACK;
      if (newPreferred != m_preferredCamera) {
        m_preferredCamera = newPreferred;
        m_lastCameraSwitchTime = currentTime;
      }
    }
    
    return m_preferredCamera;
  }

  /**
   * Updates the current pose estimates for the front and back of the robot using
   * data from Limelight cameras.
   *
   * @param gyroRate The current angular velocity of the robot, used to validate
   *                 the pose estimates.
   */
  public void setCurrentEstimates(AngularVelocity gyroRate) {
    // ---------------------------------------------------------------
    // OPTIMIZATION: Check NT server timestamp before parsing JSON.
    // getBotPoseEstimate_* calls gson.fromJson() which allocates heavily.
    // We read the same DoubleArrayEntry that LimelightHelpers uses (it's
    // cached internally) and compare the NT server timestamp.  If it hasn't
    // changed since our last call, the camera has no new frame -- skip the
    // full parse entirely.
    // ---------------------------------------------------------------
    String poseEntryKey = useMegaTag2 ? MEGATAG2_ENTRY : MEGATAG1_ENTRY;

    DoubleArrayEntry frontEntry = LimelightHelpers.getLimelightDoubleArrayEntry(
        Constants.VisionConstants.LIMELIGHT_FRONT_NAME, poseEntryKey);
    DoubleArrayEntry backEntry = LimelightHelpers.getLimelightDoubleArrayEntry(
        Constants.VisionConstants.LIMELIGHT_BACK_NAME, poseEntryKey);

    long frontNtTs = frontEntry.getAtomic().timestamp;
    long backNtTs  = backEntry.getAtomic().timestamp;

    PoseEstimate currentEstimateFront;
    PoseEstimate currentEstimateBack;

    double now = Timer.getFPGATimestamp();

    if (frontNtTs != m_lastFrontNtTimestamp && (now - m_lastFrontParseTime) >= VISION_MIN_PARSE_INTERVAL) {
      // New frame and enough time since last parse -- do the full JSON deserialization
      m_lastFrontNtTimestamp = frontNtTs;
      m_lastFrontParseTime = now;
      m_cachedFrontEstimate = useMegaTag2
          ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VisionConstants.LIMELIGHT_FRONT_NAME)
          : LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.VisionConstants.LIMELIGHT_FRONT_NAME);
    }
    currentEstimateFront = m_cachedFrontEstimate;

    if (backNtTs != m_lastBackNtTimestamp && (now - m_lastBackParseTime) >= VISION_MIN_PARSE_INTERVAL) {
      // New frame and enough time since last parse -- do the full JSON deserialization
      m_lastBackNtTimestamp = backNtTs;
      m_lastBackParseTime = now;
      m_cachedBackEstimate = useMegaTag2
          ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VisionConstants.LIMELIGHT_BACK_NAME)
          : LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.VisionConstants.LIMELIGHT_BACK_NAME);
    }
    currentEstimateBack = m_cachedBackEstimate;

    // Only check rejection once per camera estimate and cache the result
    boolean frontUpdateRejected = (currentEstimateFront == null) || rejectUpdate(currentEstimateFront, gyroRate);
    boolean backUpdateRejected = (currentEstimateBack == null) || rejectUpdate(currentEstimateBack, gyroRate);

    // Rate-limit telemetry output - only write debug info periodically
    m_telemetryCounter++;
    boolean doTelemetry = (m_telemetryCounter % VISION_TELEMETRY_PERIOD == 0);

    if (doTelemetry) {
      TelemetryManager.putBoolean("Front Update Rejected", frontUpdateRejected);
      TelemetryManager.putBoolean("Back Update Rejected", backUpdateRejected);

      if (currentEstimateFront != null && currentEstimateFront.pose != null) {
        TelemetryManager.putNumber("Vision/Front Tag Count", currentEstimateFront.tagCount);
        TelemetryManager.putNumber("Vision/Front Avg Tag Dist", currentEstimateFront.avgTagDist);
      } else {
        TelemetryManager.putNumber("Vision/Front Tag Count", 0);
      }

      if (currentEstimateBack != null && currentEstimateBack.pose != null) {
        TelemetryManager.putNumber("Vision/Back Tag Count", currentEstimateBack.tagCount);
        TelemetryManager.putNumber("Vision/Back Avg Tag Dist", currentEstimateBack.avgTagDist);
      } else {
        TelemetryManager.putNumber("Vision/Back Tag Count", 0);
      }
    }

    // Update Front Estimate
    if (!frontUpdateRejected) {
      lastEstimateFront = currentEstimateFront;
      frontPose = currentEstimateFront.pose;
      newFrontEstimate = true;
    }

    // Update Back Estimate
    if (!backUpdateRejected) {
      lastEstimateBack = currentEstimateBack;
      backPose = currentEstimateBack.pose;
      newBackEstimate = true;
    }
  }

  public Optional<PoseEstimate> determinePoseEstimate(AngularVelocity gyroRate) {
    setCurrentEstimates(gyroRate);

    // No valid pose estimates :(
    if (!newFrontEstimate && !newBackEstimate) {
      return Optional.empty();
    }

    // Use stable camera selection logic
    PreferredCamera selectedCamera = selectPreferredCamera(newFrontEstimate, newBackEstimate);
    
    PoseEstimate selectedEstimate = null;
    
    switch (selectedCamera) {
      case FRONT:
        selectedEstimate = lastEstimateFront;
        m_visionField.setRobotPose(lastEstimateFront.pose);
        newFrontEstimate = false;
        break;
        
      case BACK:
        selectedEstimate = lastEstimateBack;
        m_visionField.setRobotPose(lastEstimateBack.pose);
        newBackEstimate = false;
        break;
        
      case NONE:
      default:
        newFrontEstimate = false;
        newBackEstimate = false;
        return Optional.empty();
    }
    
    // Update timestamp tracking
    if (selectedEstimate != null) {
      m_lastAcceptedTimestamp = selectedEstimate.timestampSeconds;
    }
    
    // Clear the flags for the camera we didn't use
    if (selectedCamera == PreferredCamera.FRONT) {
      newBackEstimate = false;
    } else if (selectedCamera == PreferredCamera.BACK) {
      newFrontEstimate = false;
    }
    
    return Optional.ofNullable(selectedEstimate);
  }

  @Override
  public void periodic() {
  }
}