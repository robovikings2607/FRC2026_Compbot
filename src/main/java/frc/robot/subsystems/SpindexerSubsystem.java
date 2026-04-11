// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.PortUnreachableException;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utilities.ISysIdTunable;
import frc.robot.utilities.RobotLogger;
import frc.robot.utilities.SysIdBuilder;
import frc.robot.Constants.SpindexerConstants;

import static edu.wpi.first.units.Units.*;

public class SpindexerSubsystem extends SubsystemBase implements ISysIdTunable {
  /** Creates a new SpinDexerSubSystem. */
  private RobotContainer robot;
  private final TalonFX motor = new TalonFX(SpindexerConstants.MOTOR_ID);
  private SpindexerState state = SpindexerState.OFF;

  public SpindexerSubsystem(RobotContainer robot) {
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
            .withStatorCurrentLimit(Amps.of(SpindexerConstants.STATOR_LIMIT))
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(SpindexerConstants.SUPPLY_LIMIT))
            .withSupplyCurrentLowerLimit(Amps.of(SpindexerConstants.SUPPLY_LOWER_LIMIT))
            .withSupplyCurrentLimitEnable(true)
    );

    motor.getConfigurator().apply(configs);
  }

  public enum SpindexerState{
    FORWARD,
    REVERSE,
    OFF
  }

  public void setState(SpindexerState state){
    this.state = state;
  }

  public SpindexerState getState(){
    return state;
  }

  public void forwardControl() {
    motor.setVoltage(SpindexerConstants.SPEED);
  }
  
  public void reverseControl() {
    motor.setVoltage(-SpindexerConstants.SPEED);
  }
  
  public void stopMotor() {
    motor.stopMotor();
  }

  public void controlMotor(){
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
    
      default:
        stopMotor();
        break;
    }
  }

  public void updateLoggingData(){
    RobotLogger.logDouble("Spindexer/Voltage", motor.getMotorVoltage().getValueAsDouble());
    RobotLogger.logDouble("Spindexer/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    RobotLogger.logDouble("Spindexer/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    RobotLogger.logString("Spindexer/State", state.name());
  }

  private final SysIdRoutine sysIdRoutine = SysIdBuilder.buildTalonFXRoutine(
      motor, this, "spindexer", 4.5
  );    

   public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }    

  public TalonFX getMotor(){
    return motor;
  }
}
