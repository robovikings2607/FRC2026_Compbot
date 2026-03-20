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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
// import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.utilities.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  RobotContainer robot;
  FieldObject2d fieldVisionDetections, fieldVisionPose;
  FieldObject2d leftFieldVisionDetections, leftFieldVisionPose;
  FieldObject2d rightFieldVisionDetections, rightFieldVisionPose;  
  
  double yaw;
  String LEFT_LIMELIGHT_NAME = "limelight-left";
  String RIGHT_LIMELIGHT_NAME = "limelight-right";
  private AprilTagFieldLayout tagLayout;


  public LimelightSubsystem(RobotContainer robot) {
    // Switch to pipeline 0

    this.robot = robot;
    //Left
    LimelightHelpers.setPipelineIndex(LEFT_LIMELIGHT_NAME, 0);
    LimelightHelpers.SetIMUMode(LEFT_LIMELIGHT_NAME, 0);

    //Right
    LimelightHelpers.setPipelineIndex(RIGHT_LIMELIGHT_NAME, 0);
    LimelightHelpers.SetIMUMode(RIGHT_LIMELIGHT_NAME, 3);
    LimelightHelpers.SetIMUAssistAlpha(RIGHT_LIMELIGHT_NAME, 0.01);

    try {
        // This automatically loads the layout for the current year's game
        //tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        java.io.File deployDir = edu.wpi.first.wpilibj.Filesystem.getDeployDirectory();
        java.io.File fieldJsonFile = new java.io.File(deployDir, "2026-rebuilt-welded.json");
        tagLayout = new edu.wpi.first.apriltag.AprilTagFieldLayout(fieldJsonFile.toPath());

    } catch (IOException e) {
        e.printStackTrace();
    }

  }

  void log_limelight(String name, Optional<LimelightHelpers.PoseEstimate> estimate) {
    var fieldVisionDetections = robot.field.getObject("Limelight/"+name+"/visionDetections");
    var fieldVisionPose = robot.field.getObject("Limelight/"+name+"/fieldVisionPose");

    // Don't show stale data if this camera loses pose.
    if (!estimate.isPresent())
    {
      fieldVisionDetections.setPoses(Collections.emptyList());
      fieldVisionPose.setPoses(Collections.emptyList());
    }


/*     List<Pose2d> tagPoses = getTagPoses(estimate.get());     
    fieldVisionDetections.setPoses(tagPoses);
    fieldVisionPose.setPose(estimate.get().pose);  */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // yaw =
    // LimelightHelpers.getBotPose2d(LEFT_LIMELIGHT_NAME).getRotation().getDegrees();
    yaw = robot.drivetrain.getState().Pose.getRotation().getDegrees();

    // LimelightHelpers.SetRobotOrientation(RIGHT_LIMELIGHT_NAME, yaw, 0, 0, 0, 0,
    // 0);
    LimelightHelpers.SetRobotOrientation(LEFT_LIMELIGHT_NAME, yaw, 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate rightLL = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RIGHT_LIMELIGHT_NAME);
    LimelightHelpers.PoseEstimate leftLL = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LEFT_LIMELIGHT_NAME);

    leftFieldVisionDetections = robot.field.getObject("Limelight/" + LEFT_LIMELIGHT_NAME + "/visionDetections");
    leftFieldVisionPose = robot.field.getObject("Limelight/" + LEFT_LIMELIGHT_NAME + "/fieldVisionPose");

    rightFieldVisionDetections = robot.field.getObject("Limelight/" + RIGHT_LIMELIGHT_NAME + "/visionDetections");
    rightFieldVisionPose = robot.field.getObject("Limelight/" + RIGHT_LIMELIGHT_NAME + "/fieldVisionPose");

    SmartDashboard.putNumber("Limelight/" + RIGHT_LIMELIGHT_NAME + "/X", rightLL.pose.getX());
    SmartDashboard.putNumber("Limelight/" + RIGHT_LIMELIGHT_NAME + "/Y", rightLL.pose.getY());
    SmartDashboard.putNumber("Limelight/" + RIGHT_LIMELIGHT_NAME + "/Rotation",
    rightLL.pose.getRotation().getDegrees());

    SmartDashboard.putBoolean("Limelight/" + LEFT_LIMELIGHT_NAME + "/hasTargets",
    leftLL != null ? leftLL.tagCount > 0 : false);
    SmartDashboard.putNumber("Limelight/" + LEFT_LIMELIGHT_NAME + "/X", leftLL.pose.getX());
    SmartDashboard.putNumber("Limelight/" + LEFT_LIMELIGHT_NAME + "/Y", leftLL.pose.getY());
    SmartDashboard.putNumber("Limelight/" + LEFT_LIMELIGHT_NAME + "/Rotation", leftLL.pose.getRotation().getDegrees());

    LimelightHelpers.PoseEstimate bestPose = getBestPose(leftLL, rightLL);

    if (bestPose != null) {
      robot.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 999999999));
      robot.drivetrain.addVisionMeasurement(
          bestPose.pose,
          bestPose.timestampSeconds);
    }

    // System.out.println("has pose");
    List<Pose2d> tagPoses = getTagPoses(bestPose);

    leftFieldVisionDetections.setPoses(tagPoses);
    leftFieldVisionPose.setPose(bestPose.pose);
  }

  public boolean isValidUpdate(LimelightHelpers.PoseEstimate mt2){
    return mt2 != null && mt2.tagCount > 0;
  }

  public void configureCameraOffset(){
    LimelightHelpers.SetFidcuial3DOffset(LEFT_LIMELIGHT_NAME, yaw, yaw, yaw);
  }

  public List<Pose2d> getTagPoses(LimelightHelpers.PoseEstimate mt2) {

    ArrayList<Pose2d> tagPoses = new ArrayList<Pose2d>();
    
    if (tagLayout != null) {
      
      for (LimelightHelpers.RawFiducial tag : mt2.rawFiducials) {
          
          var pose3d = tagLayout.getTagPose(tag.id);
          if (pose3d.isPresent()) {
              tagPoses.add(pose3d.get().toPose2d());
          } else {

          }
      }
    }

    return tagPoses;
  }

  private LimelightHelpers.PoseEstimate getBestPose(LimelightHelpers.PoseEstimate leftCamera,
      LimelightHelpers.PoseEstimate rightCamera) {

    boolean lefteliminated = false;
    boolean righteliminated = false;
    /*
     * boolean leftHasBlackListedTags = false;
     * boolean rightHasBlackListedTags = false;
     * boolean leftHasWhiteListedTags = false;
     * boolean rightHasWhiteListedTags = false;
     */

    double minTagSize = 0.1; // This number needs to be derived from testing

    // Eliminate if there is no pose estimation
    if (!isValidUpdate(leftCamera)) {
      lefteliminated = true;
    }
    if (!isValidUpdate(rightCamera)) {
      righteliminated = true;
    }
    // Return null if neither camera has a valid pose
    if (lefteliminated && righteliminated) {
      return null;
    }

    // Eliminate poses derived from small tags, i.e., tags that are too far from the
    // robot.
    if (leftCamera.avgTagArea < minTagSize) {
      lefteliminated = true;
    }
    if (rightCamera.avgTagArea < minTagSize) {
      righteliminated = true;
    }
    // Return null if neither camera has large enough tags
    if (lefteliminated && righteliminated) {
      return null;
    }

    // Return the pose of the camera with the largest average area and the most
    // AprilTags used for pose estimation
    if (leftCamera.tagCount > rightCamera.tagCount) {
      // The left camera has more tags so we choose the left camera
      return leftCamera;
    } else if (rightCamera.tagCount > leftCamera.tagCount) {
      return rightCamera;
    } else {
      if (rightCamera.avgTagArea > leftCamera.avgTagArea) {
        return rightCamera;
      } else {
        return leftCamera;
      }
    }
  }

}
