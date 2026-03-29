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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldLocations;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
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
    if(DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get().equals(Alliance.Blue)){
      return robotPose.getX() > Units.inchesToMeters(182.11);
    }
    else{
      return robotPose.getX() < Units.inchesToMeters(469.11);
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

  public static InterpolatingDoubleTreeMap timeOfFlightInterp(){
    InterpolatingDoubleTreeMap interp = new InterpolatingDoubleTreeMap();
    
    interp.put(0.0, 0.7);
    interp.put(2.5, 0.7);
    interp.put(3.0, 0.8);
    interp.put(3.5, 0.9);
    interp.put(4.0, 1.0);
    interp.put(4.5, 1.1);
    interp.put(5.0, 1.2);
    interp.put(5.5, 1.3);
    interp.put(6.0, 1.4);

    return interp;
  }

  public static Translation2d virtualTarget(CommandSwerveDrivetrain drivetrain, Pose2d robotPose){
    Translation2d goalPose = determineShootingGoal(robotPose);
    Translation2d shooterPose = getShooterPose(robotPose);

    double distance = GeometryUtil.getTargetDistance(shooterPose, goalPose);
    double timeOfFlight = timeOfFlightInterp().get(distance);

    double virtualTargetX = goalPose.getX() - (drivetrain.getState().Speeds.vxMetersPerSecond * timeOfFlight);
    double virtualTargetY = goalPose.getY() - (drivetrain.getState().Speeds.vyMetersPerSecond * timeOfFlight);
    Translation2d virtualTarget = new Translation2d(virtualTargetX, virtualTargetY);

    return virtualTarget;
  }

  /*
   * test the angle returned from turret logic
  */
  public static double testTurretAngle1(Pose2d robotPose, Translation2d goalPose) {
    // Get the robot heading from the odometer robot pose.  Extract the robot heading from the robot pose.
    double robotRotation = robotPose.getRotation().getDegrees();
    // Get the shooter pose.  The shooter is not in the center of the robot so the (x,y) coordinates of the
    // shooter needs to be estimated from the robot pose.
    Translation2d shooterPose = ShooterUtils.getShooterPose(robotPose);
 
    // Estimate the heading of the turret in degrees.  Zero degrees is towards the front of the robot.
    // Positive angles are counter clockwise (CCW) and negative angles are clockwise (CW)
    double newTurretAngle = getTurretSetPoint(shooterPose, goalPose, robotRotation);

    // Check if the new set point of the turret is beyond the limits.  If not beyond the limits,
    // then no change to the new set position.  But if the new setup position is beyond the limits,
    // the turret is rotated towards the other limit in the opposite direction of travel.  The turret has physical
    // retrictions and it won't rotate infinitely.
    // This is done this way because the MIN and MAX angles are not separated by 360 degrees.  The separation
    // is less than 360 degrees.  The calculations below take that into consideration and avoid a new set position
    // in the gap between MIN and MAX angles.


    System.out.println("Turret Angle before wrap logic: " + newTurretAngle);
    
    if (newTurretAngle > (TurretConstants.MAX_ANGLE)) {
      newTurretAngle -= 360;
      if (newTurretAngle < TurretConstants.MIN_ANGLE) {
        newTurretAngle = TurretConstants.MIN_ANGLE;
      }
    } else if (newTurretAngle < (TurretConstants.MIN_ANGLE)) {
      newTurretAngle += 360;
      if (newTurretAngle > TurretConstants.MAX_ANGLE) {
        newTurretAngle = TurretConstants.MAX_ANGLE;
      }
    }

    return newTurretAngle;
  }

    /*
   * test the angle returned from turret logic
  */
  public static double testTurretAngle2(Pose2d robotPose, Translation2d goalPose) {
        
    Translation2d fieldRelativeTarget = goalPose;

    Translation2d turretFieldPosition = getTurretFieldPosition(robotPose);

    // 2. Vector Math: Calculate the line from the Turret to the Target
    Translation2d turretToTargetVector = fieldRelativeTarget.minus(turretFieldPosition);

    Rotation2d fieldAngleToTarget = turretToTargetVector.getAngle();

    Rotation2d robotRelativeAngle = fieldAngleToTarget.minus(robotPose.getRotation());

    Rotation2d motorTargetAngle = robotRelativeAngle.minus(Rotation2d.fromDegrees(0));

    double optimizedTargetDegrees = MathUtil.inputModulus(
        motorTargetAngle.getDegrees(), 
        -180.0, 
        180.0
    );

    System.out.println("Turret Angle before wrap logic: " + optimizedTargetDegrees);

    if (optimizedTargetDegrees < TurretConstants.MIN_ANGLE) {
      optimizedTargetDegrees += 360;
    }

    // Clamp the setpoint to your physical soft limits 
    double safelyClampedAngleDegrees = MathUtil.clamp(optimizedTargetDegrees, TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE);

    return safelyClampedAngleDegrees;
  }

  // This method returns the angle that the turret must be set to point at the center of the hub
  // The return angle is in degrees
  private static double getTurretSetPoint(Translation2d turretCenter, Translation2d hubCenter, double robotRotation) {
    double angle = GeometryUtil.getTargetAngle(turretCenter, hubCenter);
    angle -= robotRotation;
    return angle;
  }

  public static Translation2d getTurretFieldPosition(Pose2d robotPose) {
    return getTurretPose(robotPose).getTranslation();
  }

  public static Pose2d getTurretPose(Pose2d robotPose) {

    Pose2d turretPose = GeometryUtil.getOffsetPose(
      robotPose,
      ShooterConstants.BOT_TO_SHOOTER_DISTANCE,
      new Rotation2d(ShooterConstants.BOT_TO_SHOOTER_ANGLE)
    );    

    return turretPose;
  }

}
