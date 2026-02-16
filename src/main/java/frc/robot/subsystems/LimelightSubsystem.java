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
  boolean doRejectUpdate = false;
  // PiCameraSubsystem photonVision;
  RobotContainer m_robot;
  Pigeon2 gyro;
  FieldObject2d fieldVisionDetections, fieldVisionPose;
  double yaw;
  // AprilTagFieldLayout fieldLayout;
  int hasTargets = 0;

  public LimelightSubsystem(RobotContainer robot) {
    // Switch to pipeline 0

    m_robot = robot;
    LimelightHelpers.setPipelineIndex("limelight-left", 0);
    LimelightHelpers.SetIMUMode("limelight-left", 0);
    // LimelightHelpers.SetIMUMode("", 2);
    //  gyro = new Pigeon2(TunerConstants.kPigeonId, "drivetrain");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  yaw = m_robot.drivetrain.getState().Pose.getRotation().getDegrees();
  // yaw = m_robot.drivetrain.getPigeon2().getYaw().getValueAsDouble();

  LimelightHelpers.SetRobotOrientation("limelight-left", yaw, 0, 0, 0, 0, 0);
  LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
  
  fieldVisionDetections = m_robot.field.getObject("Limelight"+"/visionDetections");
  fieldVisionPose = m_robot.field.getObject("Limelight"+"/fieldVisionPose");

    
   if(mt2 != null && mt2.tagCount > 0){
       m_robot.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.1,0.1, 999999999));
      m_robot.drivetrain.addVisionMeasurement(
        mt2.pose,
        mt2.timestampSeconds
      ); 

      SmartDashboard.putNumber("Limelight/X", mt2.pose.getX());
      SmartDashboard.putNumber("Limelight/Y", mt2.pose.getY());
      SmartDashboard.putNumber("Limelight/Rotation", mt2.pose.getRotation().getDegrees()); 

      // System.out.println("has pose");
    }

    drawTargetsOnField(mt2);

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
}