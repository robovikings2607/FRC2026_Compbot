// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldLocations;
import frc.robot.Constants.LimelightConstants;
import frc.robot.RobotContainer;
import frc.robot.utilities.LimelightHelpers;


public class LimelightSubsystemExp extends SubsystemBase {
  RobotContainer m_robot;
  FieldObject2d fieldVisionDetections, fieldVisionPose;

  public LimelightSubsystemExp(RobotContainer robot) {

    m_robot = robot;

    //Left
    configureLLLeftOffsets();        
    LimelightHelpers.setPipelineIndex(LimelightConstants.LEFT_CAMERA_NAME, 0);
    LimelightHelpers.SetIMUMode(LimelightConstants.LEFT_CAMERA_NAME, 0);

    //Right
    configureLLRightOffsets();    
    LimelightHelpers.setPipelineIndex(LimelightConstants.RIGHT_CAMERA_NAME, 0);
    LimelightHelpers.SetIMUMode(LimelightConstants.RIGHT_CAMERA_NAME, 0);
  }

  @Override
  public void periodic() {

    fieldVisionDetections = m_robot.field.getObject("Limelight"+"/visionDetections");
    fieldVisionPose = m_robot.field.getObject("Limelight"+"/fieldVisionPose");

    Pose2d currentOdometryPose = m_robot.drivetrain.getState().Pose;
    double currentSpinRate = Math.abs(m_robot.drivetrain.getState().Speeds.omegaRadiansPerSecond);

    double yaw = currentOdometryPose.getRotation().getDegrees();

    LimelightHelpers.SetRobotOrientation(LimelightConstants.RIGHT_CAMERA_NAME, yaw, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(LimelightConstants.LEFT_CAMERA_NAME, yaw, 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate rightLL = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.RIGHT_CAMERA_NAME);
    LimelightHelpers.PoseEstimate leftLL = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.LEFT_CAMERA_NAME);

    if (isValidVisionMeasurement(rightLL)) {
      Matrix<N3, N1> stdDevs = calculateStdDevs(rightLL, currentOdometryPose, currentSpinRate );
      updateRobotPose(rightLL, stdDevs, LimelightConstants.RIGHT_CAMERA_NAME);
    }

    if (isValidVisionMeasurement(leftLL)) {
      Matrix<N3, N1> stdDevs = calculateStdDevs(leftLL, currentOdometryPose, currentSpinRate );      
      updateRobotPose(leftLL, stdDevs, LimelightConstants.LEFT_CAMERA_NAME);    
    }
  }

  private void updateRobotPose(
    LimelightHelpers.PoseEstimate mt2, 
    Matrix<N3, N1> stdDevs,
    String cameraName) {

    m_robot.drivetrain.setVisionMeasurementStdDevs(stdDevs);
    m_robot.drivetrain.addVisionMeasurement(
      mt2.pose,
      mt2.timestampSeconds
    ); 

    SmartDashboard.putNumber("Limelight/" + cameraName + "/X", mt2.pose.getX());
    SmartDashboard.putNumber("Limelight/" + cameraName + "/Y", mt2.pose.getY());
    SmartDashboard.putNumber("Limelight/" + cameraName + "/Rotation", mt2.pose.getRotation().getDegrees()); 

    drawTargetsOnField(mt2);    
  }

  private boolean isValidVisionMeasurement(LimelightHelpers.PoseEstimate mt2) {
    return mt2 != null && mt2.tagCount > 0;
  }

  /**
   * Dynamically calculates the standard deviation (trust) matrix based on area and tag count.
   */

  private Matrix<N3, N1> calculateStdDevs(
      LimelightHelpers.PoseEstimate estimate, 
      Pose2d currentOdometryPose,
      double currentSpinRate) {

      double xyStdDev = 9999999;
      double thetaStdDev = 9999999;
      
      double jumpDistance = currentOdometryPose.getTranslation().getDistance(estimate.pose.getTranslation());
      
      if (estimate.tagCount >= 2) {
          // Multi-tag tracking eliminates ambiguity, so we trust it much more than single-tag.
          // However, we still degrade trust as the average distance to the tags increases.
          
          double avgDist = estimate.avgTagDist;

          // Base StdDev (0.1 is very trusted) + (Distance Squared * Small Scaling Factor)
          // E.g., at 1 meter: 0.1 + (1 * 0.02) = 0.12 (Extremely confident)
          // E.g., at 5 meters: 0.1 + (25 * 0.02) = 0.60 (Less confident, rely more on wheel odometry)
          xyStdDev = 0.1 + (Math.pow(avgDist, 2) * 0.02); 
          
          // Heading is very stable with multiple tags, but still scales with distance
          thetaStdDev = 0.1 + (Math.pow(avgDist, 2) * 0.02); 
      } else {
          double tagArea = estimate.rawFiducials[0].ta;

          boolean isLargeTag = tagArea > 0.015; // You can tune this percentage

          // If it's a large tag OR the jump is small, AND we aren't spinning too fast
          if ((isLargeTag || jumpDistance <= 1.0) && currentSpinRate < (4.0 * Math.PI)) {
              
              if (tagArea > 0.0) {
                  xyStdDev = 0.5 + (0.005 / tagArea);
              } 
            
              thetaStdDev = 9999999; 
          }
      }

      return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

  public void drawTargetsOnField(LimelightHelpers.PoseEstimate mt2)
  {
      fieldVisionDetections.setPoses(mt2.pose);
      fieldVisionPose.setPose(mt2.pose); 
  }

  public void configureLLRightOffsets() {
      LimelightHelpers.setCameraPose_RobotSpace(
        LimelightConstants.RIGHT_CAMERA_NAME, 
        LimelightConstants.RIGHT_LL_FORWARD_OFFSET_METERS, 
        LimelightConstants.RIGHT_LL_RIGHT_OFFSET_METERS, 
        LimelightConstants.RIGHT_LL_UP_OFFSET_METERS, 
        LimelightConstants.RIGHT_LL_ROLL_OFFSET_DEGREES, 
        LimelightConstants.RIGHT_LL_PITCH_OFFSET_DEGREES, 
        LimelightConstants.RIGHT_LL_YAW_OFFSET_DEGREES);
  }  

  public void configureLLLeftOffsets() {

      LimelightHelpers.setCameraPose_RobotSpace(
        LimelightConstants.LEFT_CAMERA_NAME, 
        LimelightConstants.LEFT_LL_FORWARD_OFFSET_METERS, 
        LimelightConstants.LEFT_LL_RIGHT_OFFSET_METERS, 
        LimelightConstants.LEFT_LL_UP_OFFSET_METERS, 
        LimelightConstants.LEFT_LL_ROLL_OFFSET_DEGREES, 
        LimelightConstants.LEFT_LL_PITCH_OFFSET_DEGREES, 
        LimelightConstants.LEFT_LL_YAW_OFFSET_DEGREES);
  }  

  public void analyzeMultipleTargets(String cameraName) {
      // 1. Fetch the full parsed JSON dump from the Limelight
      LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(cameraName);

      // 2. Ensure we have valid data by checking the new .valid boolean
      if (results != null && results.valid && results.targets_Fiducials != null) {
          
          // 3. Loop through the array DIRECTLY from the results object
          for (LimelightHelpers.LimelightTarget_Fiducial tag : results.targets_Fiducials) {
              
              int tagID = (int) tag.fiducialID;
              
              // --- THE THREE MOST USEFUL POSES ---
              
              // A. Where the tag is relative to the center of your robot chassis
              Pose3d poseInRobotSpace = tag.getTargetPose_RobotSpace();
              
              // B. Where the tag is relative to the camera lens itself
              Pose3d poseInCameraSpace = tag.getTargetPose_CameraSpace();
              
              // C. Where your robot is on the field, calculated using ONLY this specific tag
              Pose3d robotPoseFieldSpace = tag.getRobotPose_FieldSpace();
              
              // Example: Calculate the true 3D straight-line distance to this specific tag
              double distanceToTagMeters = poseInRobotSpace.getTranslation().getNorm();
              
              System.out.println("Tag " + tagID + " is " + distanceToTagMeters + " meters away.");
          }
      }
  }
}