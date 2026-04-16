// Copyback (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.RobotLogger;

public class LimelightSubsystem extends SubsystemBase {

  // -------------------------------------------------------------------------
  // Vision filtering constants (tunable)
  // -------------------------------------------------------------------------

  /** Reject single-tag estimates with pose ambiguity above this threshold. */
  private static final double kAmbiguityThreshold = 1e9; //0.3

  /**
   * Minimum average tag area (% of image) required for single-tag estimates.
   * Multi-tag estimates use the lower kMinTagAreaMulti threshold.
   */
  private static final double kMinTagAreaSingle = 0.2;
  private static final double kMinTagAreaMulti  = 0.05;

  private static final double kMaxTagArea = 4.9;

  /**
   * Reject any pose whose translation norm is below this value.
   * Catches bad solves that report a position near the field origin.
   */
  private static final double kMinPoseNorm = 0.5; // meters

  /**
   * Base XY standard deviation for multi-tag and single-tag estimates (meters).
   * Scaled up linearly with average tag distance.
   */
  private static final double kMultiTagBaseStd  = 0.2;
  private static final double kSingleTagBaseStd = 0.4;

  /** Per-meter distance scaling factor applied to base stddev. */
  private static final double kDistStdScale = 0.05;

  /** Large variance used for the rotation component — we trust the gyro, not vision heading. */
  private static final double kLargeVariance = 1e9;

  // -------------------------------------------------------------------------
  // Camera names
  // -------------------------------------------------------------------------
  private final String FRONT_LIMELIGHT_NAME  = "limelight-front";
  private final String BACK_LIMELIGHT_NAME = "limelight-back";

  // -------------------------------------------------------------------------
  // State
  // -------------------------------------------------------------------------
  RobotContainer robot;

  Pose2d frontFieldVisionPose;
  Pose2d[] frontFieldVisionDetections;
  Pose2d backFieldVisionPose;
  Pose2d[] backFieldVisionDetections;


  double yaw;
  private AprilTagFieldLayout tagLayout;

  /** Timestamp of the last measurement submitted to the pose estimator. */
  private double lastSubmittedTimestamp = 0.0;

  // -------------------------------------------------------------------------
  // Inner type
  // -------------------------------------------------------------------------
  private static class CameraEstimate {
    final Pose2d pose;
    final double timestampSeconds;
    final Matrix<N3, N1> stdDevs;
    final int tagCount;

    CameraEstimate(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs, int tagCount) {
      this.pose             = pose;
      this.timestampSeconds = timestampSeconds;
      this.stdDevs          = stdDevs;
      this.tagCount         = tagCount;
    }
  }

  // -------------------------------------------------------------------------
  // Constructor
  // -------------------------------------------------------------------------
  public LimelightSubsystem(RobotContainer robot) {
    this.robot = robot;

    LimelightHelpers.setPipelineIndex(FRONT_LIMELIGHT_NAME, 0);
    LimelightHelpers.SetIMUMode(FRONT_LIMELIGHT_NAME, 0);

    LimelightHelpers.setPipelineIndex(BACK_LIMELIGHT_NAME, 0);
    LimelightHelpers.SetIMUMode(BACK_LIMELIGHT_NAME, 0);
    //LimelightHelpers.SetIMUAssistAlpha(BACK_LIMELIGHT_NAME, 0.01);

    try {
      java.io.File deployDir = edu.wpi.first.wpilibj.Filesystem.getDeployDirectory();
      java.io.File fieldJsonFile = new java.io.File(deployDir, "2026-rebuilt-welded.json");
      tagLayout = new edu.wpi.first.apriltag.AprilTagFieldLayout(fieldJsonFile.toPath());
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  // -------------------------------------------------------------------------
  // Periodic
  // -------------------------------------------------------------------------
  @Override
  public void periodic() {
    yaw = robot.drivetrain.getState().Pose.getRotation().getDegrees();

    LimelightHelpers.PoseEstimate frontMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(FRONT_LIMELIGHT_NAME);
    LimelightHelpers.PoseEstimate backMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(BACK_LIMELIGHT_NAME);

    //double frontHeading = (Math.abs(yaw - frontMT1.pose.getRotation().getDegrees()) > 40) ? yaw : frontMT1.pose.getRotation().getDegrees();
    //%double backHeading = (Math.abs(yaw - backMT1.pose.getRotation().getDegrees()) > 40) ? yaw : backMT1.pose.getRotation().getDegrees();

    LimelightHelpers.SetRobotOrientation(FRONT_LIMELIGHT_NAME,  yaw, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(BACK_LIMELIGHT_NAME, yaw, 0, 0, 0, 0, 0);

    LimelightHelpers.PoseEstimate frontRaw  = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(FRONT_LIMELIGHT_NAME);
    LimelightHelpers.PoseEstimate backRaw = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(BACK_LIMELIGHT_NAME);

    RobotLogger.logBoolean("Limelight/" + FRONT_LIMELIGHT_NAME  + "/hasTargets", frontRaw  != null && frontRaw.tagCount  > 0);
    RobotLogger.logBoolean("Limelight/" + BACK_LIMELIGHT_NAME + "/hasTargets", backRaw != null && backRaw.tagCount > 0);    

    Optional<CameraEstimate> frontEst  = processCamera(frontRaw,  FRONT_LIMELIGHT_NAME);
    Optional<CameraEstimate> backEst = processCamera(backRaw, BACK_LIMELIGHT_NAME);

    if (frontEst.isPresent()) {
      frontFieldVisionPose = frontRaw.pose;
      frontFieldVisionDetections = getTagPoses(frontRaw).toArray(Pose2d[]::new);
    } else {
      frontFieldVisionPose = Constants.EMPTY_POSE;
      frontFieldVisionDetections =  new Pose2d[]{};
    }
    
    updateFieldVisualization(frontFieldVisionPose,  frontFieldVisionDetections,  FRONT_LIMELIGHT_NAME);

    if (backEst.isPresent()) {
      backFieldVisionPose = backRaw.pose;
      backFieldVisionDetections = getTagPoses(backRaw).toArray(Pose2d[]::new);
    } else {
      backFieldVisionPose = Constants.EMPTY_POSE;
      backFieldVisionDetections =  new Pose2d[]{};
    }

    updateFieldVisualization(backFieldVisionPose,  backFieldVisionDetections,  BACK_LIMELIGHT_NAME);

    // Fuse both cameras before submitting a single update to the pose estimator.
    // This avoids two correlated updates hitting the Kalman filter independently.
    Optional<CameraEstimate> toSubmit;
    if (frontEst.isPresent() && backEst.isPresent()) {
      toSubmit = Optional.of(fuseEstimates(frontEst.get(), backEst.get()));
    } else if (frontEst.isPresent()) {
      toSubmit = frontEst;
    } else {
      toSubmit = backEst;
    }

    toSubmit.ifPresent(est -> {
      if (est.timestampSeconds > lastSubmittedTimestamp) {
        robot.drivetrain.addVisionMeasurement(est.pose, est.timestampSeconds, est.stdDevs);
        lastSubmittedTimestamp = est.timestampSeconds;
        RobotLogger.logDouble("Limelight/" + "Robot/fusedtimestamped", est.timestampSeconds);                   
        RobotLogger.logStruct("Limelight/" + "Robot/robotPose", Pose2d.struct, est.pose);           
      }
    });
  }

  // -------------------------------------------------------------------------
  // Per-camera filtering
  // -------------------------------------------------------------------------

  /**
   * Applies 254-inspired filtering to a raw MegaTag2 estimate.
   *
   * <p>Checks applied:
   * <ol>
   *   <li>Null / zero-tag guard</li>
   *   <li>Single-tag: ambiguity threshold</li>
   *   <li>Single-tag: minimum tag area</li>
   *   <li>Multi-tag: minimum tag area</li>
   *   <li>Pose norm check (rejects solves near the field origin)</li>
   *   <li>Timestamp deduplication</li>
   * </ol>
   *
   * <p>Standard deviations are scaled with average tag distance so that
   * farther (less reliable) observations receive proportionally less weight
   * in the Kalman filter.
   */
  private Optional<CameraEstimate> processCamera(LimelightHelpers.PoseEstimate mt, String name) {
    if (mt == null || mt.tagCount == 0) {
      RobotLogger.logBoolean("Limelight/" + name + "/no Tags", true);                         
      return Optional.empty();
    }
    RobotLogger.logBoolean("Limelight/" + name + "/no Tags", false);                             

    if (mt.timestampSeconds <= lastSubmittedTimestamp) {
      RobotLogger.logBoolean("Limelight/" + name + "/bad Timestamp", true);                                   
      return Optional.empty();
    }
    RobotLogger.logBoolean("Limelight/" + name + "/bad Timestamp", false);                                       

    if (mt.tagCount < 2) {
      // Single-tag extra checks
      if (mt.rawFiducials != null) {
        for (LimelightHelpers.RawFiducial f : mt.rawFiducials) {
          if (f.ambiguity > kAmbiguityThreshold) {
            RobotLogger.logBoolean("Limelight/" + name + "/bad Ambuguity", true);                                                   
            return Optional.empty();
          }
        }
      }
      RobotLogger.logBoolean("Limelight/" + name + "/bad Ambuguity", false);                                                         

      if (mt.avgTagArea < kMinTagAreaSingle || mt.avgTagArea > kMaxTagArea) {
        RobotLogger.logBoolean("Limelight/" + name + "/bad Area", true);                                                                 
        return Optional.empty();
      }
      RobotLogger.logBoolean("Limelight/" + name + "/bad Area", false);                                                                       
    } 
    else {
      if (mt.avgTagArea < kMinTagAreaMulti) {
        RobotLogger.logBoolean("Limelight/" + name + "/bad MinTagAreaMulti", true);                                                                               
        return Optional.empty();
      }
      RobotLogger.logBoolean("Limelight/" + name + "/bad MinTagAreaMulti", false);                                                                                     
    }

    // Reject poses suspiciously close to the field origin (likely a bad solve)
    if (mt.pose.getTranslation().getNorm() < kMinPoseNorm) {
      RobotLogger.logBoolean("Limelight/" + name + "/bad PoseNorm", true);                                                                                           
      return Optional.empty();
    }
    RobotLogger.logBoolean("Limelight/" + name + "/bad PoseNorm", false);                                                                                               

    // Quality-scaled standard deviations.
    // quality = 1.0 for multi-tag; 1.0 - ambiguity for single-tag (per 254's metric).
    // stddev scales with 1/quality so higher ambiguity → less trust.
    // An additional distance term accounts for tag size / measurement noise at range.
    double quality = (mt.tagCount >= 2) ? 1.0
        : 1.0 - mt.rawFiducials[0].ambiguity;
    double baseStd = (mt.tagCount >= 2) ? kMultiTagBaseStd : kSingleTagBaseStd;
    double xyStd   = baseStd * (1.0 / quality) * (1.0 + kDistStdScale * mt.avgTagDist);
    Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStd, xyStd, kLargeVariance);
    //Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStd, xyStd, Units.degreesToRadians(90)); //n3 is in degrees, I believe

    return Optional.of(new CameraEstimate(mt.pose, mt.timestampSeconds, stdDevs, mt.tagCount));
  }

  // -------------------------------------------------------------------------
  // Inverse-variance weighted fusion of two camera estimates
  // -------------------------------------------------------------------------

  /**
   * Fuses two per-camera estimates into one using inverse-variance weighting,
   * matching 254's approach in VisionSubsystem.fuseEstimates().
   *
   * <p>Because both cameras share the same gyro via MegaTag2, we keep the
   * rotation from the more recent estimate and only fuse translation.
   */
  private CameraEstimate fuseEstimates(CameraEstimate a, CameraEstimate b) {
    // Work in variance space
    double varAx = sq(a.stdDevs.get(0, 0));
    double varAy = sq(a.stdDevs.get(1, 0));
    double varBx = sq(b.stdDevs.get(0, 0));
    double varBy = sq(b.stdDevs.get(1, 0));

    double wAx = 1.0 / varAx, wBx = 1.0 / varBx;
    double wAy = 1.0 / varAy, wBy = 1.0 / varBy;

    Pose2d fusedPose = new Pose2d(
        new Translation2d(
            (a.pose.getX() * wAx + b.pose.getX() * wBx) / (wAx + wBx),
            (a.pose.getY() * wAy + b.pose.getY() * wBy) / (wAy + wBy)),
        // Both rotations are gyro-derived via MT2; use the newer one
        (a.timestampSeconds >= b.timestampSeconds)
            ? a.pose.getRotation()
            : b.pose.getRotation());

    Matrix<N3, N1> fusedStdDevs = VecBuilder.fill(
        Math.sqrt(1.0 / (wAx + wBx)),
        Math.sqrt(1.0 / (wAy + wBy)),
        kLargeVariance);

    double fusedTimestamp = Math.max(a.timestampSeconds, b.timestampSeconds);
    int fusedTagCount     = a.tagCount + b.tagCount;

    return new CameraEstimate(fusedPose, fusedTimestamp, fusedStdDevs, fusedTagCount);
  }

  private static double sq(double x) { return x * x; }

  // -------------------------------------------------------------------------
  // Helpers
  // -------------------------------------------------------------------------

  private void updateFieldVisualization(
      Pose2d robotPose,
      Pose2d[] aprilTagPoses,
      String name) {

      RobotLogger.logStruct("Limelight/" + name + "/RobotPose", Pose2d.struct, robotPose);    
      RobotLogger.logStructArray("Limelight/" + name + "/AprilTagPoses", Pose2d.struct, aprilTagPoses);
  }

  public boolean isValidUpdate(LimelightHelpers.PoseEstimate mt2) {
    return mt2 != null && mt2.tagCount > 0;
  }

  public void configureCameraOffset() {
    LimelightHelpers.SetFidcuial3DOffset(FRONT_LIMELIGHT_NAME, yaw, yaw, yaw);
  }

  public List<Pose2d> getTagPoses(LimelightHelpers.PoseEstimate mt2) {
    ArrayList<Pose2d> tagPoses = new ArrayList<>();
    if (tagLayout != null && mt2.rawFiducials != null) {
      for (LimelightHelpers.RawFiducial tag : mt2.rawFiducials) {
        var pose3d = tagLayout.getTagPose(tag.id);
        if (pose3d.isPresent()) {
          tagPoses.add(pose3d.get().toPose2d());
        }
      }
    }
    return tagPoses;
  }

  private LimelightHelpers.PoseEstimate getBestPose(
      LimelightHelpers.PoseEstimate frontCamera, LimelightHelpers.PoseEstimate backCamera) {

    boolean fronteliminated  = false;
    boolean backeliminated = false;

    double minTagSize = 0.1;

    if (isValidUpdate(frontCamera))  { fronteliminated  = true; }
    if (isValidUpdate(backCamera)) { backeliminated = true; }
    if (fronteliminated && backeliminated) { return null; }

    if (frontCamera.avgTagArea  < minTagSize) { fronteliminated  = true; }
    if (backCamera.avgTagArea < minTagSize) { backeliminated = true; }
    if (fronteliminated && backeliminated) { return null; }

    if (frontCamera.tagCount > backCamera.tagCount) {
      return frontCamera;
    } else if (backCamera.tagCount > frontCamera.tagCount) {
      return backCamera;
    } else {
      return (backCamera.avgTagArea > frontCamera.avgTagArea) ? backCamera : frontCamera;
    }
  }

  private boolean hasBlackListedTags(LimelightHelpers.PoseEstimate cameraPose) {
    int[] blackList = {7, 6, 17};
    for (int i = 0; i < cameraPose.rawFiducials.length; i++) {
      for (int j = 0; j < blackList.length; j++) {
        if (cameraPose.rawFiducials[i].id == blackList[j]) { return true; }
      }
    }
    return false;
  }

  private boolean hasWhiteListedTags(LimelightHelpers.PoseEstimate cameraPose) {
    int[] whiteList = {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27};
    for (int i = 0; i < cameraPose.rawFiducials.length; i++) {
      for (int j = 0; j < whiteList.length; j++) {
        if (cameraPose.rawFiducials[i].id == whiteList[j]) { return true; }
      }
    }
    return false;
  }
}