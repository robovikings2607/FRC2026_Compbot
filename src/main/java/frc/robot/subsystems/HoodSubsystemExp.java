// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.utilities.GeometryUtil;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.HoodConstants;


public class HoodSubsystemExp extends ShooterComponentSubsystemExp {
  private static final double GEAR_RATIO = 1.0;
  private final double TARGET_ERR_TOLERANCE_ROTATIONS = 0.01;  

  
  public HoodSubsystemExp(RobotContainer robot) {
    super(robot, HoodConstants.HOOD_ID);    
    
    configureMotor();
  }

   /**
   * Sets the angle of motor directly assuming the correct angle
   * is already known and does not have to be looked up in the map
   */
  public void setAngle(double targetAngleDegrees) {
    double motorSetpointRotations = GeometryUtil.getDegreesAsMotorRotations(targetAngleDegrees, GEAR_RATIO);
    
    SetMotorPosition(motorSetpointRotations, "Hood/newSetPointRotations");    
  }


  

  @Override
  public void periodic() {
  }

  @Override
  protected void configureMotor(){

    TalonFXConfiguration configs = new TalonFXConfiguration();

    var slot0Configs = configs.Slot0;
        slot0Configs.kS = 0.25; // Voltage output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps requires this voltage output.
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires this voltage output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations requires this voltage output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.11; // A velocity error of 1 rps requires this voltage output

    var motionMagicConfigs = configs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 160; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 320; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

/*     //enable software limits
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    //limits (in rotations)
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = rotationsPerDegree * 120;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -rotationsPerDegree * 240; */
  
    motor.getConfigurator().apply(configs);
    motor.setNeutralMode(NeutralModeValue.Brake);

    motor.setPosition(0); //
  }

  @Override
  public double getTargetTolerance() {
    return TARGET_ERR_TOLERANCE_ROTATIONS;
  }

}




