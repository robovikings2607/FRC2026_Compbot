// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.PortUnreachableException;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utilities.ISysIdTunable;
import frc.robot.utilities.RobotLogger;
import frc.robot.utilities.SysIdBuilder;
import frc.robot.Constants.FeederConstants;

import static edu.wpi.first.units.Units.*;

public class FeederSubsystem extends SubsystemBase implements ISysIdTunable {
  /** Creates a new FeederSubsystem. */
  private RobotContainer robot;
  private final TalonFX motor = new TalonFX(FeederConstants.MOTOR_ID);
  private final TalonFXConfiguration configs = new TalonFXConfiguration();
  private final VelocityVoltage pid = new VelocityVoltage(0.0);
  private final CoastOut coastOut = new CoastOut();
  private FeederState state = FeederState.FORWARD;
  private double goal;

  public FeederSubsystem(RobotContainer robot) {
    this.robot = robot;

    configureMotor();
    //configurePID();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateLoggingData();
  }

  public void configureMotor(){
    configs.withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(FeederConstants.STATOR_LIMIT))
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(FeederConstants.SUPPLY_LIMIT))
            .withSupplyCurrentLowerLimit(Amps.of(FeederConstants.SUPPLY_LOWER_LIMIT))
            .withSupplyCurrentLimitEnable(true)
    );

    Slot0Configs slot0Configs = configs.Slot0;
          slot0Configs.kS = FeederConstants.S; // Voltage output to overcome static friction
          slot0Configs.kV = FeederConstants.V; // A velocity target of 1 rps requires this voltage output.
          slot0Configs.kP = FeederConstants.P; // A position error of 2.5 rotations requires this voltage output
          slot0Configs.kI = FeederConstants.I; // no output for integrated error
          slot0Configs.kD = FeederConstants.D; // A velocity error of 1 rps requires this voltage output

    configs.withSlot0(slot0Configs);

    motor.getConfigurator().apply(configs);
  }

  public void configurePID(){}

  public enum FeederState{
    FORWARD,
    REVERSE,
    DYNAMIC,
    PID_TUNING,
    OFF
  }

  public void setState(FeederState state){
    this.state = state;
  }

  public FeederState getState(){
    return state;
  }

  public void forwardControl() {
    goal = FeederConstants.SPEED;
    motor.setControl(pid.withVelocity(goal));
  }

  public void reverseControl(){
    goal = -FeederConstants.SPEED;
    motor.setControl(pid.withVelocity(goal));
  }

  public void dynamicControl(){
    goal = FeederConstants.SPEED * robot.flywheel.percentOfGoal();
    motor.setControl(pid.withVelocity(goal));
  }

  public void PIDTuningControl(){
    configs.Slot0.kS = SmartDashboard.getNumber("Feeder/Tuning/PID/S", 0);
    configs.Slot0.kV = SmartDashboard.getNumber("Feeder/Tuning/PID/V", 0);
    configs.Slot0.kP = SmartDashboard.getNumber("Feeder/Tuning/PID/P", 0);
    configs.Slot0.kI = SmartDashboard.getNumber("Feeder/Tuning/PID/I", 0);
    configs.Slot0.kD = SmartDashboard.getNumber("Feeder/Tuning/PID/D", 0);
    motor.getConfigurator().apply(configs);
    goal = SmartDashboard.getNumber("Feeder/Tuning/Goal(RPS)", 0.0);  
    motor.setControl(pid.withVelocity(goal));
  }

  public void coastOut() {
    motor.setControl(coastOut);
  }

  public void controlMotor(){
    switch (state) {
      case FORWARD:
        forwardControl();
        break;

      case REVERSE:
        reverseControl();
        break;

      case DYNAMIC:
        dynamicControl();
        break;

      case PID_TUNING:
        PIDTuningControl();
        break;

      case OFF:
        coastOut();
        break;
    
      default:
        coastOut();
        break;
    }
  }

  public void updateLoggingData(){
    RobotLogger.logDouble("Feeder/CurrentRPS", motor.getVelocity().getValueAsDouble());
    RobotLogger.logDouble("Feeder/Goal(rps)", goal);
    RobotLogger.logDouble("Feeder/Voltage", motor.getMotorVoltage().getValueAsDouble());
    RobotLogger.logDouble("Feeder/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    RobotLogger.logDouble("Feeder/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    RobotLogger.logString("Feeder/State", state.name());
  }

  public void createTuningData(){
    RobotLogger.logDouble("Feeder/Tuning/PID/S", 0);
    RobotLogger.logDouble("Feeder/Tuning/PID/V", 0);
    RobotLogger.logDouble("Feeder/Tuning/PID/P", 0);
    RobotLogger.logDouble("Feeder/Tuning/PID/I", 0);
    RobotLogger.logDouble("Feeder/Tuning/PID/D", 0);
    RobotLogger.logDouble("Feeder/Tuning/Goal(rps)", 0);
  }

  private final SysIdRoutine sysIdRoutine = SysIdBuilder.buildTalonFXRoutine(
        motor, this, "feeder", 4.0
  );    

  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }    

  public TalonFX getMotor(){
    return motor;
  }
}