// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.KickerConstants;
import frc.robot.commands.intake.PulseKicker;
import frc.robot.utilities.RobotLogger;

import static edu.wpi.first.units.Units.*;

public class KickerSubsystem extends SubsystemBase {
  /** Creates a new KickerSubsystem. */
  private RobotContainer robot;
  private final TalonFX motor = new TalonFX(KickerConstants.MOTOR_ID);
  private KickerState state = KickerState.OFF;

  public KickerSubsystem(RobotContainer robot) {
    this.robot = robot;
  
    configureMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateLoggingData();
  }

  public void configureMotor(){
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.withCurrentLimits(
        new CurrentLimitsConfigs()
            // Swerve azimuth does not require much torque output, so we can set a relatively low
            // stator current limit to help avoid brownouts without impacting performance.
            .withStatorCurrentLimit(Amps.of(KickerConstants.STATOR_LIMIT))
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(KickerConstants.SUPPLY_LIMIT))
            .withSupplyCurrentLowerLimit(Amps.of(KickerConstants.SUPPLY_LOWER_LIMIT))
            .withSupplyCurrentLimitEnable(true)
    );

    motor.getConfigurator().apply(configs);
  }

  public enum KickerState{
    FORWARD,
    SHOOTING,
    REVERSE,
    OFF
  }

  public void setState(KickerState state){
    this.state = state;
  }

  public KickerState getState(){
    return state;
  }

  public void forwardControl(){
    motor.setVoltage(KickerConstants.SPEED);
  }
  
  public void reverseControl(){
    motor.setVoltage(-KickerConstants.SPEED);
  }

  public void shootingControl(){
    motor.setVoltage(3.0);
  }

  public void stopMotor(){
    motor.stopMotor();
  }

  public void controlMotor(KickerState state){
    this.state = state;

    switch (state) {
      case FORWARD:
        forwardControl();
        break;      
      case REVERSE:
        reverseControl();
        break;

      case OFF:
        stopMotor();
        break;

      case SHOOTING:
        shootingControl();
        break;
    
      default:
        stopMotor();
        break;
    }
  }

  public void updateLoggingData(){
    /* RobotLogger.logDouble("Kicker/Voltage", motor.getMotorVoltage().getValueAsDouble());
    RobotLogger.logDouble("Kicker/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    RobotLogger.logDouble("Kicker/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble()); */
    RobotLogger.logString("Kicker/State", state.name());
  }

  public TalonFX getMotor(){
    return motor;
  }
}
