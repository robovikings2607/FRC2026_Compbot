// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utilities.ShooterUtils;
import frc.robot.Constants.FieldElements;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */
  private TalonFX hoodMotor;
  private RobotContainer robot;

  private final MotionMagicVoltage magicMotionRequest = new MotionMagicVoltage(0);
  private static final double gearRatio = ((350.0/50.0)*(26.0/12.0));
  private static final double rotationsPerDegree = gearRatio/360.0;
  private double setPoint, goal;
  private InterpolatingDoubleTreeMap hoodInterp = new InterpolatingDoubleTreeMap();


  public HoodSubsystem(RobotContainer robot) {
    this.robot = robot;

    configureMotor();
    createInterpMap();
    SmartDashboard.putNumber("Hood/SetPoint", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Pose2d robotPose = robot.drivetrain.getState().Pose;

    Translation2d shooterPose = ShooterUtils.getShooterPose(robotPose);
    Translation2d goalPose = new Translation2d();

    if(DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get().equals(Alliance.Blue)){
      goalPose = FieldElements.BLUE_HUB;
    }
    else {
      goalPose = FieldElements.RED_HUB;            
    }

    double distance = shooterPose.getDistance(goalPose);
    SmartDashboard.putNumber("Hood/Distance", distance);
    setGoal(distance);

    // setPoint = SmartDashboard.getNumber("Hood/SetPoint", 0) * rotationsPerDegree;

    hoodMotor.setControl(magicMotionRequest.withPosition(setPoint));
  }


  public void configureMotor(){
    hoodMotor = new TalonFX(HoodConstants.HOOD_ID);

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

     //enable software limits
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    //limits (in rotations)
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HoodConstants.MIN_HOOD_ANGLE * rotationsPerDegree;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HoodConstants.MAX_HOOD_ANGLE * rotationsPerDegree; 
  
    hoodMotor.getConfigurator().apply(configs);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);

    hoodMotor.setPosition(0);
  }

  public void createInterpMap(){
    //key = distance from goal
    //value = position of hood in desired shot angle
    hoodInterp.put(0.0, 0.0);
    hoodInterp.put(2.4, -3.0);
    hoodInterp.put(3.03, -3.0);
    hoodInterp.put(3.51, -4.0);
    hoodInterp.put(4.03, 0.0);
    hoodInterp.put(5.5, -3.0);
  }

  public void setGoal(double distance){
    goal = 90.0 - hoodInterp.get(distance);
    setPoint = goal * rotationsPerDegree;
  }

  public void zeroMotor(){
    hoodMotor.set(0.2);

    if(hoodMotor.getStatorCurrent().getValueAsDouble() > 20){
      hoodMotor.set(0);
      hoodMotor.setPosition(HoodConstants.MIN_HOOD_ANGLE * rotationsPerDegree);
    }
  }

  public double getGoal(){
    return goal;
  }

  public double getSetPoint(){
    return setPoint;
  }
}