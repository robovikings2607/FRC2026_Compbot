// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public final class ShooterUtils {

 public static Translation2d getShooterPose(Pose2d robotPose){    
 /*    double robotRotation = MathUtil.inputModulus(robotPose.getRotation().getDegrees(), -180, 180);

    double shooterX = robotPose.getX() 
              + Units.inchesToMeters(ShooterConstants.BOT_TO_SHOOTER_DISTANCE)
              * Math.cos(Math.PI + ShooterConstants.BOT_TO_SHOOTER_ANGLE + Math.toRadians(robotRotation));
    double shooterY = robotPose.getY()
              + Units.inchesToMeters(ShooterConstants.BOT_TO_SHOOTER_DISTANCE)
              * Math.sin(Math.PI + ShooterConstants.BOT_TO_SHOOTER_ANGLE + Math.toRadians(robotRotation)); 
    return new Translation2d(shooterX, shooterY); */

    return GeometryUtil.getOffsetPose(robotPose, 
                                      ShooterConstants.BOT_TO_SHOOTER_DISTANCE, 
                                      new Rotation2d(ShooterConstants.BOT_TO_SHOOTER_ANGLE)
                                      ).getTranslation();
  }

  public static double getRPS(double RPM) {
    return RPM / 60;
  }
}
