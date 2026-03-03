// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldLocations;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;

/** Add your docs here. */
public final class ShooterUtils {

 public static Translation2d getShooterPose(Pose2d robotPose){    
    double robotRotation = MathUtil.inputModulus(robotPose.getRotation().getDegrees(), -180, 180);

    double shooterX = robotPose.getX() 
              + Units.inchesToMeters(ShooterConstants.BOT_TO_SHOOTER_DISTANCE)
              * Math.cos(Math.PI + ShooterConstants.BOT_TO_SHOOTER_ANGLE + Math.toRadians(robotRotation));
    double shooterY = robotPose.getY()
              + Units.inchesToMeters(ShooterConstants.BOT_TO_SHOOTER_DISTANCE)
              * Math.sin(Math.PI + ShooterConstants.BOT_TO_SHOOTER_ANGLE + Math.toRadians(robotRotation)); 
    return new Translation2d(shooterX, shooterY);

/*    return GeometryUtil.getOffsetPose(robotPose, 
                                      ShooterConstants.BOT_TO_SHOOTER_DISTANCE, 
                                      new Rotation2d(ShooterConstants.BOT_TO_SHOOTER_ANGLE)
                                      ).getTranslation(); */
  }

  public static boolean inNeutralZone(Pose2d robotPose){
    return robotPose.getX() > Units.inchesToMeters(182.11) && robotPose.getX() < Units.inchesToMeters(469.11);
  }

  public static Translation2d determineShootingGoal(Pose2d robotPose){
    Translation2d goalPose = new Translation2d();

    if(DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get().equals(Alliance.Blue)){ //Blue/No Alliance
      if(ShooterUtils.inNeutralZone(robotPose)){ //Ferry Mode
        if(robotPose.getY() < Units.inchesToMeters(158.845)){
            goalPose = FieldLocations.BLUE_RIGHT_FERRY_POINT;
          }
        else{
            goalPose = FieldLocations.BLUE_LEFT_FERRY_POINT;
        }
      }
      else{ //Shooting Mode
        goalPose = FieldLocations.BLUE_HUB;
      }
    }
    else { //Red Alliance
      if(ShooterUtils.inNeutralZone(robotPose)){ //Ferry Mode
        if(robotPose.getY() < Units.inchesToMeters(159.845)){
            goalPose = FieldLocations.RED_LEFT_FERRY_POINT;
          }
        else{
            goalPose = FieldLocations.RED_RIGHT_FERRY_POINT;
        }
      }
      else{ //Shooting Mode
        goalPose = FieldLocations.RED_HUB;
      }    
    }

    return goalPose;
  }

  public static InterpolatingDoubleTreeMap timeOfFlightInterp(){
    InterpolatingDoubleTreeMap interp = new InterpolatingDoubleTreeMap();
    
    interp.put(0.0, null);
    interp.put(2.53, null);
    interp.put(3.1, null);
    interp.put(3.5, null);
    interp.put(4.0, null);
    interp.put(4.5, null);
    interp.put(5.0, null);
    interp.put(5.5, null);
    interp.put(6.0, null);

    return interp;
  }

  public static Translation2d virtualTarget(RobotContainer robot){
    CommandSwerveDrivetrain drivetrain = robot.drivetrain;

    Pose2d robotPose = drivetrain.getState().Pose;
    Translation2d goalPose = determineShootingGoal(robotPose);

    double distance = GeometryUtil.getTargetDistance(robotPose.getTranslation(), goalPose);
    double timeOfFlight = timeOfFlightInterp().get(distance);

    double virtualTargetX = goalPose.getX() - (drivetrain.getState().Speeds.vxMetersPerSecond * timeOfFlight);
    double virtualTargetY = goalPose.getY() - (drivetrain.getState().Speeds.vyMetersPerSecond * timeOfFlight);
    Translation2d virtualTarget = new Translation2d(virtualTargetX, virtualTargetY);

    return virtualTarget;
  }

  public static double getRPS(double RPM) {
    return RPM / 60;
  }
}
