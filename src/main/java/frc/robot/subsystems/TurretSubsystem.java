// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.AllPermission;
import java.util.Collections;
import java.util.List;

import org.opencv.core.Mat;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldLocations;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.utilities.GeometryUtil;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.RobotLogger;
import frc.robot.utilities.ShooterUtils;

import static edu.wpi.first.units.Units.*;

public class TurretSubsystem extends SubsystemBase {
  private static final double rotationsPerDegree = 10.0/360.0;
  private final TalonFX turretMotor;
  private final RobotContainer robot;
  private MotionMagicVoltage magicMotionRequest;
  private PositionVoltage positionVoltage;
  private boolean fixedShot = false;
  private boolean isDeactivated = false;

  public TurretSubsystem(RobotContainer robot) {
    this.robot = robot;

    turretMotor = new TalonFX(TurretConstants.TURRET_ID);

    //turretMotor.setPosition(0);
    //previousSetPoint = turretMotor.getPosition().getValueAsDouble();
        
    // turretMotor.setPosition(0.0);

    configureMotor();
    magicMotionRequest = new MotionMagicVoltage(0.0);
    positionVoltage = new PositionVoltage(0.0);

    logNumber2("Turret/BootUpPose", turretMotor.getPosition().getValueAsDouble());
    logNumber2("SetOffset", 0.0);    
    //logNumber2("Turret/MotorCurrent", turretMotor.getStatorCurrent().getValueAsDouble());        
  }

  private void configureMotor() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    var slot0Configs = configs.Slot0;
        slot0Configs.kS = 1.0; // Voltage output to overcome static friction
        //slot0Configs.kV = 0.12; // A velocity target of 1 rps requires this voltage output.
        //slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires this voltage output
        slot0Configs.kP = 6.0; // A position error of 2.5 rotations requires this voltage output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.15; // A velocity error of 1 rps requires this voltage output

    /* var motionMagicConfigs = configs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 70; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 2800; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 112000; // Target jerk of 1600 rps/s/s (0.1 seconds) */

    //enable software limits
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    //limits (in rotations)
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = rotationsPerDegree * TurretConstants.MAX_ANGLE;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = rotationsPerDegree * TurretConstants.MIN_ANGLE; 
  
    configs.withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );
  
    turretMotor.getConfigurator().apply(configs);
    turretMotor.setNeutralMode(NeutralModeValue.Brake);    
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run

    // Get the robot pose from the odometer.  These are the robot (x,y) coordinates and robot reading.
    // These are the coordinates of the center of the robot.
    Pose2d robotPose = robot.drivetrain.getState().Pose;
    // Get the robot heading from the odometer robot pose.  Extract the robot heading from the robot pose.
    double robotRotation = robotPose.getRotation().getDegrees();
    // Get the shooter pose.  The shooter is not in the center of the robot so the (x,y) coordinates of the
    // shooter needs to be estimated from the robot pose.
    Translation2d shooterPose = ShooterUtils.getShooterPose(robotPose);
    // This method returns a virtual shooting target taking into consideration the location of the robot in
    // the field and the velocity of the robot to compensate for on-the-move shooting
    Translation2d goalPose = ShooterUtils.virtualTarget(robot.drivetrain, robotPose);

    // Estimate the heading of the turret in degrees.  Zero degrees is towards the front of the robot.
    // Positive angles are counter clockwise (CCW) and negative angles are clockwise (CW)
    double newSetPoint = getTurretSetPoint(shooterPose, goalPose, robotRotation);

    // Check if the new set point of the turret is beyond the limits.  If not beyond the limits,
    // then no change to the new encoder position.  But if the new setup position is beyond the limits,
    // the turret is rotated by 360 degrees in the opposite direction of travel.  The turret has physical
    // retrictions and it won't rotate infinitely.
    double newEncoderPos = newSetPoint * rotationsPerDegree;

    if(newSetPoint > (TurretConstants.MAX_ANGLE)){
      newEncoderPos -= 360 * rotationsPerDegree;
    }
    else if(newSetPoint < (TurretConstants.MIN_ANGLE)){
      newEncoderPos += 360 * rotationsPerDegree;
    }

    // This allows for adjustment of the turret offset angle in degrees.
    double offset = SmartDashboard.getNumber("Turret/Offset", 0.0);

    // Set the motor to the new encoder position if the turret has not been deactivated
    if(isDeactivated){
      turretMotor.setControl(new CoastOut());
    }
    else{
      turretMotor.setControl(positionVoltage.withPosition(newEncoderPos));
    }

    // Rumble the driver controller if the turret is getting close to the limits, both the minimum and
    // maximum limit.
    if(turretMotor.getPosition().getValueAsDouble() > (TurretConstants.MAX_ANGLE - 3.0) * rotationsPerDegree ||
       turretMotor.getPosition().getValueAsDouble() < (TurretConstants.MIN_ANGLE + 3.0) * rotationsPerDegree){
        robot.driverController.controller.setRumble(GenericHID.RumbleType.kBothRumble, 1);
       }
    else{
      robot.driverController.controller.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }

    // Write key values to network tables
    logNumber("Turret/NewSetPoint", newSetPoint);        
    logNumber("Turret/NewPosition", newEncoderPos);        
    logNumber("Turret/ActualPosition", turretMotor.getPosition().getValueAsDouble());

    // Write key values to the log
    logNumber2("Turret/NewSetPoint", newSetPoint);        
    logNumber2("Turret/NewPosition", newEncoderPos);        
    logNumber2("Turret/ActualPosition", turretMotor.getPosition().getValueAsDouble());        
  }

  // This method returns the angle that the turret must be set to point at the center of the hub
  // The return angle is in degrees
  private static double getTurretSetPoint(Translation2d turretCenter, Translation2d hubCenter, double robotRotation) {
    double angle = GeometryUtil.getTargetAngle(turretCenter, hubCenter);
    double robotRotationAdjustedAngle = MathUtil.inputModulus(angle - robotRotation, -180.0, 180.0);   

    return -robotRotationAdjustedAngle;
  }

  public void fixedShot(boolean fixed){
    fixedShot = fixed;
  }

  public void setTurretToZero(){
    turretMotor.setPosition(0);
  }

/*   public Command zeroTurret(){
    return run(() -> turretMotor.set(0.2))
           .until(() -> isPressed)
           .andThen(() -> turretMotor.set(0.0))
           .andThen(() -> isZeroed())
           .andThen(() -> setTurretToZero());
  } */

  public boolean inTolerance(double pose){
    return turretMotor.getPosition().getValueAsDouble() > pose - 0.5 ||
           turretMotor.getPosition().getValueAsDouble() < pose + 0.5; 
  }

  public void deactivateTurret(boolean deactivate){
    isDeactivated = deactivate;
  }

  public TalonFX getMotor(){
    return turretMotor;
  }

  public void logNumber(String key, double value){
    SmartDashboard.putNumber(key, value);
  }

  public void logNumber2(String key, double value){
    RobotLogger.logDouble(key, value);
  }
}
