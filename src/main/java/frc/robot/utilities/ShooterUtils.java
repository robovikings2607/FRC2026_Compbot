// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldLocations;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;

/** Add your docs here. */
public final class ShooterUtils {

  static double updateDelay = 0.02; //tune
  
   public static Translation2d getShooterPose(Pose2d robotPose){    
      double robotRotation = MathUtil.inputModulus(robotPose.getRotation().getDegrees(), -180, 180);
  
      double shooterX = robotPose.getX() 
                + ShooterConstants.BOT_TO_SHOOTER_DISTANCE
                * Math.cos(Math.PI + ShooterConstants.BOT_TO_SHOOTER_ANGLE + Math.toRadians(robotRotation));
      double shooterY = robotPose.getY()
                + ShooterConstants.BOT_TO_SHOOTER_DISTANCE
                * Math.sin(Math.PI + ShooterConstants.BOT_TO_SHOOTER_ANGLE + Math.toRadians(robotRotation)); 
      return new Translation2d(shooterX, shooterY);
  
  /*    return GeometryUtil.getOffsetPose(robotPose, 
                                        ShooterConstants.BOT_TO_SHOOTER_DISTANCE, 
                                        new Rotation2d(ShooterConstants.BOT_TO_SHOOTER_ANGLE)
                                        ).getTranslation(); */
    }
  
    public static boolean inNeutralZone(Pose2d robotPose){
      if(DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get().equals(Alliance.Blue)){
        return robotPose.getX() > Units.inchesToMeters(182.11);
      }
      else{
        return robotPose.getX() < Units.inchesToMeters(469.11);
      }
    }    
    
    public static boolean inOppoAllianceZone(Pose2d robotPose){
      if(DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get().equals(Alliance.Blue)){
        return robotPose.getX() > Units.inchesToMeters(469.11);
      }
      else{
        return robotPose.getX() < Units.inchesToMeters(182.11);
      }
    }
  
    public static Translation2d determineShootingGoal(Pose2d robotPose){
      Translation2d goalPose = new Translation2d();
  
     if(DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get().equals(Alliance.Blue)){ //Blue/No Alliance
        if(ShooterUtils.inNeutralZone(robotPose)){ //Ferry Mode
          if(robotPose.getY() > Units.inchesToMeters(158.845)){
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
          if(robotPose.getY() > Units.inchesToMeters(158.845)){
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
  
    public static final InterpolatingDoubleTreeMap timeOfFlightInterp = new InterpolatingDoubleTreeMap();

    static {
      timeOfFlightInterp.put(0.0, 1.0);
      timeOfFlightInterp.put(1.5, 1.0);
      timeOfFlightInterp.put(2.0, 1.15);
      timeOfFlightInterp.put(2.5, 1.2);
      timeOfFlightInterp.put(3.0, 1.25);
      timeOfFlightInterp.put(3.5, 1.25);
      timeOfFlightInterp.put(4.0, 1.2);
      timeOfFlightInterp.put(4.5, 1.5);
      timeOfFlightInterp.put(5.0, 1.5);
      timeOfFlightInterp.put(5.5, 1.6);
      timeOfFlightInterp.put(6.0, 1.7);
    }
  
    public static Translation2d virtualTarget(CommandSwerveDrivetrain drivetrain, Pose2d robotPose){
      Translation2d goalPose = determineShootingGoal(robotPose);
      Translation2d shooterPose = getShooterPose(robotPose);
  
      double distance = GeometryUtil.getTargetDistance(shooterPose, goalPose);
      double timeOfFlight = timeOfFlightInterp.get(distance);

      double virtualTargetX = goalPose.getX() - (drivetrain.getState().Speeds.vyMetersPerSecond * timeOfFlight);
      double virtualTargetY = goalPose.getY() - (drivetrain.getState().Speeds.vxMetersPerSecond * timeOfFlight);
      Translation2d virtualTarget = new Translation2d(virtualTargetX, virtualTargetY);
  
      return virtualTarget;
    }
  
    public static Translation2d stuypulesShootOnMove(CommandSwerveDrivetrain drivetrain, Pose2d robotPose){
      Translation2d goalPose = determineShootingGoal(robotPose);
      // Translation2d shooterPose = getShooterPose(robotPose);
      Translation2d shooterPose = futureShooterPose(drivetrain, robotPose, true);
      Translation2d vituralPose = goalPose;
  
      double distance = shooterPose.getDistance(goalPose);
      double timeOfFlight = timeOfFlightInterp.get(distance);
  
      // double vx = drivetrain.getState().Speeds.vxMetersPerSecond;
      // double vy = drivetrain.getState().Speeds.vyMetersPerSecond;
  
      double[] v = tangentialVelocities(drivetrain, robotPose, true);
      double vx = v[0];
      double vy = v[1];
  
      for(int i = 0; i < 10; i++){
        double dx = vx * timeOfFlight;
        double dy = vy * timeOfFlight;
  
        vituralPose = new Translation2d(goalPose.getX() - dx, goalPose.getY() - dy);
        distance = shooterPose.getDistance(vituralPose);
  
        if(Math.abs(timeOfFlightInterp.get(distance) - timeOfFlight) < 0.01){
          break;
        }
  
        timeOfFlight = timeOfFlightInterp.get(distance);
      }
  
      return vituralPose;
    }
  
    public static Translation2d futureShooterPose(CommandSwerveDrivetrain drivetrain, Pose2d robotPose, boolean useAccel){
      ChassisSpeeds robotRelativeChassisSpeeds = drivetrain.getState().Speeds;
  
      Pigeon2 gyro = drivetrain.getPigeon2();
      double ax = gyro.getAccelerationX().getValueAsDouble() * 9.81;
      double ay = gyro.getAccelerationY().getValueAsDouble() * 9.81;
  
      double omega = robotRelativeChassisSpeeds.omegaRadiansPerSecond;
  
      Pose2d futureRobotPose = robotPose.exp(
        new Twist2d(
          robotRelativeChassisSpeeds.vxMetersPerSecond * updateDelay,
          robotRelativeChassisSpeeds.vyMetersPerSecond * updateDelay,
        omega * updateDelay
      ));

    if(useAccel){
      futureRobotPose = robotPose.exp(
        new Twist2d (
            robotRelativeChassisSpeeds.vxMetersPerSecond * updateDelay + 0.5 * ax * updateDelay*updateDelay,
            robotRelativeChassisSpeeds.vyMetersPerSecond * updateDelay + 0.5 * ay * updateDelay*updateDelay,
            omega * updateDelay
        )
      );
    }

    return getShooterPose(futureRobotPose);
  }

  public static double[] tangentialVelocities(CommandSwerveDrivetrain drivetrain, Pose2d robotPose, boolean useAccel){
    ChassisSpeeds robotRelativeChassisSpeeds = drivetrain.getState().Speeds;
    ChassisSpeeds fieldRelativeChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeChassisSpeeds, 
                                                                                    drivetrain.getState().Pose.getRotation());
    
    double[] velocities = new double[2];

    Translation2d shooterPose = getShooterPose(robotPose);
        
    double omega = robotRelativeChassisSpeeds.omegaRadiansPerSecond;

    Pigeon2 gyro = drivetrain.getPigeon2();
    double ax = gyro.getAccelerationX().getValueAsDouble() * 9.81;
    double ay = gyro.getAccelerationY().getValueAsDouble() * 9.81;

    Translation2d r = shooterPose.minus(robotPose.getTranslation());

    double vx = fieldRelativeChassisSpeeds.vxMetersPerSecond - omega * r.getY();
    double vy = fieldRelativeChassisSpeeds.vyMetersPerSecond + omega * r.getX();

    if(useAccel){
      Translation2d fieldAccel = new Translation2d(ax, ay).rotateBy(robotPose.getRotation());

      vx += fieldAccel.getX() * updateDelay;
      vy += fieldAccel.getY() * updateDelay;
    }

    velocities[0] = vx;
    velocities[1] = vy;

    return velocities;
  }
}
