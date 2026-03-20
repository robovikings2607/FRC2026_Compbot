// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.utilities.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {

  // -------------------------------------------------------------------------
  // Vision filtering constants (tunable)
  // -------------------------------------------------------------------------

  /** Reject single-tag estimates with pose ambiguity above this threshold. */
  private static final double kAmbiguityThreshold = 0.3;

  /**
   * Minimum average tag area (% of image) required for single-tag estimates.
   * Multi-tag estimates use the lower kMinTagAreaMulti threshold.
   */
  private static final double kMinTagAreaSingle = 0.2;
  private static final double kMinTagAreaMulti  = 0.05;

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
  private final String LEFT_LIMELIGHT_NAME  = "limelight-left";
  private final String RIGHT_LIMELIGHT_NAME = "limelight-right";

  // -------------------------------------------------------------------------
  // State
  // -------------------------------------------------------------------------
  RobotContainer robot;
  FieldObject2d leftFieldVisionDetections,  leftFieldVisionPose;
  FieldObject2d rightFieldVisionDetections, rightFieldVisionPose;

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

    LimelightHelpers.setPipelineIndex(LEFT_LIMELIGHT_NAME, 0);
    LimelightHelpers.SetIMUMode(LEFT_LIMELIGHT_NAME, 0);

    LimelightHelpers.setPipelineIndex(RIGHT_LIMELIGHT_NAME, 0);
    LimelightHelpers.SetIMUMode(RIGHT_LIMELIGHT_NAME, 3);
    LimelightHelpers.SetIMUAssistAlpha(RIGHT_LIMELIGHT_NAME, 0.01);

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

    LimelightHelpers.SetRobotOrientation(LEFT_LIMELIGHT_NAME,  yaw, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(RIGHT_LIMELIGHT_NAME, yaw, 0, 0, 0, 0, 0);

    LimelightHelpers.PoseEstimate leftRaw  = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LEFT_LIMELIGHT_NAME);
    LimelightHelpers.PoseEstimate rightRaw = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RIGHT_LIMELIGHT_NAME);

    leftFieldVisionDetections  = robot.field.getObject("Limelight/" + LEFT_LIMELIGHT_NAME  + "/visionDetections");
    leftFieldVisionPose        = robot.field.getObject("Limelight/" + LEFT_LIMELIGHT_NAME  + "/fieldVisionPose");
    rightFieldVisionDetections = robot.field.getObject("Limelight/" + RIGHT_LIMELIGHT_NAME + "/visionDetections");
    rightFieldVisionPose       = robot.field.getObject("Limelight/" + RIGHT_LIMELIGHT_NAME + "/fieldVisionPose");

    SmartDashboard.putBoolean("Limelight/" + LEFT_LIMELIGHT_NAME  + "/hasTargets", leftRaw  != null && leftRaw.tagCount  > 0);
    SmartDashboard.putBoolean("Limelight/" + RIGHT_LIMELIGHT_NAME + "/hasTargets", rightRaw != null && rightRaw.tagCount > 0);

    Optional<CameraEstimate> leftEst  = processCamera(leftRaw,  LEFT_LIMELIGHT_NAME);
    Optional<CameraEstimate> rightEst = processCamera(rightRaw, RIGHT_LIMELIGHT_NAME);

    // Field2d visualization
    updateFieldVisualization(leftEst,  leftRaw,  leftFieldVisionDetections,  leftFieldVisionPose,  LEFT_LIMELIGHT_NAME);
    updateFieldVisualization(rightEst, rightRaw, rightFieldVisionDetections, rightFieldVisionPose, RIGHT_LIMELIGHT_NAME);

    // Fuse both cameras before submitting a single update to the pose estimator.
    // This avoids two correlated updates hitting the Kalman filter independently.
    Optional<CameraEstimate> toSubmit;
    if (leftEst.isPresent() && rightEst.isPresent()) {
      toSubmit = Optional.of(fuseEstimates(leftEst.get(), rightEst.get()));
    } else if (leftEst.isPresent()) {
      toSubmit = leftEst;
    } else {
      toSubmit = rightEst;
    }

    toSubmit.ifPresent(est -> {
      if (est.timestampSeconds > lastSubmittedTimestamp) {
        robot.drivetrain.addVisionMeasurement(est.pose, est.timestampSeconds, est.stdDevs);
        lastSubmittedTimestamp = est.timestampSeconds;
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
      return Optional.empty();
    }

    if (mt.timestampSeconds <= lastSubmittedTimestamp) {
      return Optional.empty();
    }

    if (mt.tagCount < 2) {
      // Single-tag extra checks
      if (mt.rawFiducials != null) {
        for (LimelightHelpers.RawFiducial f : mt.rawFiducials) {
          if (f.ambiguity > kAmbiguityThreshold) {
            return Optional.empty();
          }
        }
      }
      if (mt.avgTagArea < kMinTagAreaSingle) {
        return Optional.empty();
      }
    } else {
      if (mt.avgTagArea < kMinTagAreaMulti) {
        return Optional.empty();
      }
    }

    // Reject poses suspiciously close to the field origin (likely a bad solve)
    if (mt.pose.getTranslation().getNorm() < kMinPoseNorm) {
      return Optional.empty();
    }

    // Quality-scaled standard deviations.
    // quality = 1.0 for multi-tag; 1.0 - ambiguity for single-tag (per 254's metric).
    // stddev scales with 1/quality so higher ambiguity → less trust.
    // An additional distance term accounts for tag size / measurement noise at range.
    double quality = (mt.tagCount >= 2) ? 1.0
        : 1.0 - mt.rawFiducials[0].ambiguity;
    double baseStd = (mt.tagCount >= 2) ? kMultiTagBaseStd : kSingleTagBaseStd;
    double xyStd   = baseStd * (1.0 / quality) * (1.0 + kDistStdScale * mt.avgTagDist);
    Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStd, xyStd, kLargeVariance);

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
      Optional<CameraEstimate> est,
      LimelightHelpers.PoseEstimate raw,
      FieldObject2d detections,
      FieldObject2d poseObj,
      String name) {

    if (!est.isPresent()) {
      detections.setPoses(Collections.emptyList());
      poseObj.setPoses(Collections.emptyList());
      return;
    }

    poseObj.setPose(est.get().pose);
    if (raw != null) {
      detections.setPoses(getTagPoses(raw));
    }

    SmartDashboard.putNumber("Limelight/" + name + "/X",        est.get().pose.getX());
    SmartDashboard.putNumber("Limelight/" + name + "/Y",        est.get().pose.getY());
    SmartDashboard.putNumber("Limelight/" + name + "/Rotation", est.get().pose.getRotation().getDegrees());
  }

  public boolean isValidUpdate(LimelightHelpers.PoseEstimate mt2) {
    return mt2 != null && mt2.tagCount > 0;
  }

  public void configureCameraOffset() {
    LimelightHelpers.SetFidcuial3DOffset(LEFT_LIMELIGHT_NAME, yaw, yaw, yaw);
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
      LimelightHelpers.PoseEstimate leftCamera, LimelightHelpers.PoseEstimate rightCamera) {

    boolean lefteliminated  = false;
    boolean righteliminated = false;

    double minTagSize = 0.1;

    if (isValidUpdate(leftCamera))  { lefteliminated  = true; }
    if (isValidUpdate(rightCamera)) { righteliminated = true; }
    if (lefteliminated && righteliminated) { return null; }

    if (leftCamera.avgTagArea  < minTagSize) { lefteliminated  = true; }
    if (rightCamera.avgTagArea < minTagSize) { righteliminated = true; }
    if (lefteliminated && righteliminated) { return null; }

    if (leftCamera.tagCount > rightCamera.tagCount) {
      return leftCamera;
    } else if (rightCamera.tagCount > leftCamera.tagCount) {
      return rightCamera;
    } else {
      return (rightCamera.avgTagArea > leftCamera.avgTagArea) ? rightCamera : leftCamera;
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
