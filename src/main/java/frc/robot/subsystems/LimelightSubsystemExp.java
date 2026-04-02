// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.RobotContainer;
import frc.robot.utilities.LimelightHelpers;


public class LimelightSubsystemExp extends SubsystemBase {
  RobotContainer m_robot;
  FieldObject2d fieldVisionDetections, fieldVisionPose;
  IVisionOdometryUpdater odometryUpdateStrategy;

  public LimelightSubsystemExp(RobotContainer robot, IVisionOdometryUpdater odometryUpdateStrategy) {

    m_robot = robot;
    this.odometryUpdateStrategy = odometryUpdateStrategy;

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

    double yaw = m_robot.drivetrain.getState().Pose.getRotation().getDegrees();

    LimelightHelpers.SetRobotOrientation(LimelightConstants.RIGHT_CAMERA_NAME, yaw, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(LimelightConstants.LEFT_CAMERA_NAME, yaw, 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate rightLL = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.RIGHT_CAMERA_NAME);
    LimelightHelpers.PoseEstimate leftLL = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.LEFT_CAMERA_NAME);

    drawTargetsOnField(rightLL);    
    drawTargetsOnField(leftLL);    

    odometryUpdateStrategy.updateRobotPoseFromVision(m_robot.drivetrain, rightLL, leftLL);
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