// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Collections;

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
  double yaw;
  String LEFT_LIMELIGHT_NAME = "limelight-left";
  String RIGHT_LIMELIGHT_NAME = "limelight-right";

  public LimelightSubsystem(RobotContainer robot) {
    // Switch to pipeline 0

    this.robot = robot;
    //Left
    LimelightHelpers.setPipelineIndex(LEFT_LIMELIGHT_NAME, 0);
    LimelightHelpers.SetIMUMode(LEFT_LIMELIGHT_NAME, 0);

    //Right
    LimelightHelpers.setPipelineIndex(RIGHT_LIMELIGHT_NAME, 0);
    LimelightHelpers.SetIMUMode(RIGHT_LIMELIGHT_NAME, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  yaw = robot.drivetrain.getState().Pose.getRotation().getDegrees();

  LimelightHelpers.SetRobotOrientation(RIGHT_LIMELIGHT_NAME, yaw, 0, 0, 0, 0, 0);
  LimelightHelpers.SetRobotOrientation(LEFT_LIMELIGHT_NAME, yaw, 0, 0, 0, 0, 0);
  LimelightHelpers.PoseEstimate rightLL = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RIGHT_LIMELIGHT_NAME);
  LimelightHelpers.PoseEstimate leftLL = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LEFT_LIMELIGHT_NAME);

  
  fieldVisionDetections = robot.field.getObject("Limelight"+"/visionDetections");
  fieldVisionPose = robot.field.getObject("Limelight"+"/fieldVisionPose");

    
   if(isValidUpdate(leftLL)){
      LimelightHelpers.PoseEstimate mt2 = leftLL;

      robot.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.1,0.1, 999999999));
      robot.drivetrain.addVisionMeasurement(
        mt2.pose,
        mt2.timestampSeconds
      ); 

      SmartDashboard.putNumber("Limelight/" + LEFT_LIMELIGHT_NAME + "/X", mt2.pose.getX());
      SmartDashboard.putNumber("Limelight/" + LEFT_LIMELIGHT_NAME + "/Y", mt2.pose.getY());
      SmartDashboard.putNumber("Limelight/" + LEFT_LIMELIGHT_NAME + "/Rotation", mt2.pose.getRotation().getDegrees()); 

      // System.out.println("has pose");
      drawTargetsOnField(mt2);
    }

    if(isValidUpdate(rightLL)){
      LimelightHelpers.PoseEstimate mt2 = rightLL;

      robot.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.1,0.1, 999999999));
      robot.drivetrain.addVisionMeasurement(
        mt2.pose,
        mt2.timestampSeconds
      ); 

      SmartDashboard.putNumber("Limelight/" + RIGHT_LIMELIGHT_NAME + "/X", mt2.pose.getX());
      SmartDashboard.putNumber("Limelight/" + RIGHT_LIMELIGHT_NAME + "/Y", mt2.pose.getY());
      SmartDashboard.putNumber("Limelight/" + RIGHT_LIMELIGHT_NAME + "/Rotation", mt2.pose.getRotation().getDegrees()); 

      // System.out.println("has pose");
      drawTargetsOnField(mt2);
    }


   //  SmartDashboard.putNumber("Limelight/hasTarget", hasTargets);
  }

  public void drawTargetsOnField(LimelightHelpers.PoseEstimate mt2)
  {
      if(mt2 == null){
          return;
      }

      if (mt2.pose.equals(new Pose2d()))
      {
          fieldVisionDetections.setPoses(Collections.emptyList());
          fieldVisionPose.setPoses(Collections.emptyList());
          return;
      }

      fieldVisionDetections.setPoses(mt2.pose);
      fieldVisionPose.setPose(mt2.pose); 
  }

  public boolean isValidUpdate(LimelightHelpers.PoseEstimate mt2){
    return mt2 != null && mt2.tagCount > 0;
  }
}