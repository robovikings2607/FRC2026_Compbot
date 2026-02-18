// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.AllPermission;

import org.opencv.core.Mat;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldElements;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.utilities.GeometryUtil;
import frc.robot.utilities.ShooterUtils;

public class TurretSubsystem extends SubsystemBase {
  private static final double rotationsPerDegree = 10.0/360.0;
  private final TalonFX turretMotor;
  private final RobotContainer robot;
  private final MotionMagicVoltage magicMotionRequest = new MotionMagicVoltage(0);
  private final DigitalInput limitSwitch;
  private double previousSetPoint, previousEncoderPos;
  private boolean isPressed, isZeroed;
  private double offset = .442;

  public TurretSubsystem(RobotContainer robot) {
    this.robot = robot;

    turretMotor = new TalonFX(TurretConstants.TURRET_ID);

    //turretMotor.setPosition(0);

    limitSwitch = new DigitalInput(0);
    isPressed = !limitSwitch.get(); //get() returns false when pressed/switched
    isZeroed = true;

    previousSetPoint = 0;
    previousEncoderPos = 0;
        
    // turretMotor.setPosition(0.0);

    configureMotor();

    SmartDashboard.putNumber("Turret/MotorCurrent", turretMotor.getStatorCurrent().getValueAsDouble());
  }

  private void configureMotor() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    var slot0Configs = configs.Slot0;
        slot0Configs.kS = 0.25; // Voltage output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps requires this voltage output.
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires this voltage output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations requires this voltage output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.11; // A velocity error of 1 rps requires this voltage output

    var motionMagicConfigs = configs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 10; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 20; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 100; // Target jerk of 1600 rps/s/s (0.1 seconds)

/*     //enable software limits
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    //limits (in rotations)
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = rotationsPerDegree * 120;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -rotationsPerDegree * 240; */
  
    turretMotor.getConfigurator().apply(configs);
    turretMotor.setNeutralMode(NeutralModeValue.Brake);    
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run

    isPressed = !limitSwitch.get();

    Pose2d robotPose = robot.drivetrain.getState().Pose;

    double robotRotation = robotPose.getRotation().getDegrees();
    Translation2d shooterPose = ShooterUtils.getShooterPose(robotPose);

    //checks alliance and aims at corresponding hub
    double newSetPoint = 0;

    if(DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get().equals(Alliance.Blue)){
      newSetPoint = getTurretSetPoint(shooterPose, FieldElements.BLUE_HUB, robotRotation);      
    }
    else {
      newSetPoint = getTurretSetPoint(shooterPose, FieldElements.BLUE_HUB, robotRotation);      
    }

    double newEncoderPos = previousEncoderPos + getDelta(previousSetPoint, newSetPoint);

    if(newEncoderPos > TurretConstants.MAX_ANGLE * rotationsPerDegree){
      newEncoderPos -= 360 * rotationsPerDegree;
    }
    else if(newEncoderPos < TurretConstants.MIN_ANGLE * rotationsPerDegree){
      newEncoderPos += 360 * rotationsPerDegree;
    }

    turretMotor.setControl(magicMotionRequest.withPosition(newEncoderPos - offset));

    SmartDashboard.putNumber("Turret/Delta", getDelta(previousSetPoint, newSetPoint));
    SmartDashboard.putNumber("Turret/PreviousSetPoint", previousSetPoint);
    SmartDashboard.putNumber("Turret/PreviousPosition", previousEncoderPos);

    previousSetPoint = newSetPoint;
    previousEncoderPos = newEncoderPos;

    SmartDashboard.putNumber("Turret/NewSetPoint", newSetPoint);
    SmartDashboard.putNumber("Turret/NewPosition", newEncoderPos);
    SmartDashboard.putBoolean("Turret/SwitchIsPressed", isPressed);
    SmartDashboard.putNumber("Turret/ActualPosition", turretMotor.getPosition().getValueAsDouble());
  }

  private static double getTurretSetPoint(Translation2d turretCenter, Translation2d hubCenter, double robotRotation) {
    double angle = GeometryUtil.getTargetAngle(turretCenter, hubCenter);
    double robotRotationAdjustedAngle = angle - robotRotation;   

    return -robotRotationAdjustedAngle * rotationsPerDegree;
  }

  private static double getDelta(double previousSetPoint, double newSetPoint){
    double delta = 0;

    if(Math.abs(previousSetPoint - newSetPoint) > 9.5){ //if turret wraps
      delta = 10 - Math.abs(previousSetPoint - newSetPoint);
     
      if(previousSetPoint < newSetPoint){
        delta = -delta;
      }
    }
    else{
      delta = newSetPoint - previousSetPoint;
    }

    return delta;
  }

  public void isZeroed(){
    isZeroed = true;
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

  public void zeroTurret(){
    if(!isZeroed){
      turretMotor.set(0.1375);
    }
    if(isPressed){
      turretMotor.set(0.0);
      isZeroed = true;
      turretMotor.setPosition(-1.39);
    }
  }

  public boolean inTolerance(double pose){
    return turretMotor.getPosition().getValueAsDouble() > pose - 0.5 ||
           turretMotor.getPosition().getValueAsDouble() < pose + 0.5; 
  }

}
