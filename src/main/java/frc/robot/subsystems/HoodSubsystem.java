// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */
  private RobotContainer robot;
  private TalonSRX motor;
  private CANcoder encoder;
  private PhoenixPIDController pid;
  private InterpolatingDoubleTreeMap shootingInterp, ferryingInterp;
  private HoodStates state = HoodStates.SHOOTING;
  private double goal, output;

  public HoodSubsystem(RobotContainer robot) {
    this.robot = robot;

    configureMotor();
    configureEncoder();
    configurePID();
    createShootingInterpMap();
    createFerryingInterpMap();
    createTuningData();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateLoggingData();
  }

  public void configureMotor(){
    motor = new TalonSRX(HoodConstants.MOTOR_ID);  
  }

  public void configureEncoder(){
    encoder = new CANcoder(HoodConstants.ENCODER_ID);
    CANcoderConfiguration configs = new CANcoderConfiguration();

    //ensures no jump discontinuity occurs
    configs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.0;
    configs.MagnetSensor.MagnetOffset = HoodConstants.ENCODER_MAGNET_OFFSET;

    encoder.getConfigurator().apply(configs);
  }

  public void configurePID(){
    pid = new PhoenixPIDController(HoodConstants.P, HoodConstants.I, HoodConstants.D);
    pid.setTolerance(0.002); //quates to +- 0.05 degrees of error, may need to change
  }

  public enum HoodStates{
    SHOOTING,
    FERRYING,
    FIXED,
    TUNING
  }

  public void setState(HoodStates state){
    this.state = state;
  }

  public HoodStates getState(){
    return state;
  }

  public void createShootingInterpMap(){
    //key = distance from goal
    //value = position of hood in encoder values
    shootingInterp = new InterpolatingDoubleTreeMap();

    shootingInterp.put(0.0, 0.0);
    shootingInterp.put(6.0, 0.0);
  }
  public void createFerryingInterpMap(){
    //key = distance from goal
    //value = position of hood in encoder values
    ferryingInterp = new InterpolatingDoubleTreeMap();

    ferryingInterp.put(0.0, 0.0);
    ferryingInterp.put(6.0, 0.0);
  }

  public void setGoal(InterpolatingDoubleTreeMap interp, double distance){ 
    goal = interp.get(distance);
    MathUtil.clamp(goal, HoodConstants.ENCODER_MAX, HoodConstants.ENCODER_MIN);
  }

  public double getGoal(){
    return goal;
  }

  public void shootingControl(double distance){
    setGoal(shootingInterp, distance);
    output = -pid.calculate(encoder.getAbsolutePosition().getValueAsDouble(), goal, Timer.getFPGATimestamp());
    motor.set(TalonSRXControlMode.PercentOutput, output);
  }  
  
  public void ferryingControl(double distance){
    setGoal(ferryingInterp, distance);
    output = -pid.calculate(encoder.getAbsolutePosition().getValueAsDouble(), goal, Timer.getFPGATimestamp());
    motor.set(TalonSRXControlMode.PercentOutput, output);
  }  
  
  public void fixedControl(){
    setGoal(shootingInterp, 3.0);
    output = -pid.calculate(encoder.getAbsolutePosition().getValueAsDouble(), goal, Timer.getFPGATimestamp());
    motor.set(TalonSRXControlMode.PercentOutput, output);
  }

  public void tuningControl(){
    pid.setP(SmartDashboard.getNumber("Hood/Tuning/P", 0));
    pid.setI(SmartDashboard.getNumber("Hood/Tuning/I", 0));
    pid.setD(SmartDashboard.getNumber("Hood/Tuning/D", 0));
    goal = SmartDashboard.getNumber("Hood/Tuning/Goal", 0);

    output = -pid.calculate(encoder.getAbsolutePosition().getValueAsDouble(), goal, Timer.getFPGATimestamp());
    motor.set(TalonSRXControlMode.PercentOutput, output);
  }

  public void controlMotor(double distance){
    switch (state) {
      case SHOOTING:
        shootingControl(distance);
        break;

      case FERRYING:
        ferryingControl(distance);
        break;
      
      case FIXED:
        fixedControl();
        break;

      case TUNING:
        tuningControl();
    
      default:
        shootingControl(distance);
        break;
    }
  }

  public void updateLoggingData(){
    SmartDashboard.putNumber("Hood/Output", output);
    SmartDashboard.putNumber("Hood/Goal", goal);
    SmartDashboard.putNumber("Hood/StatorCurrent", motor.getStatorCurrent());
    SmartDashboard.putNumber("Hood/SupplyCurrent", motor.getSupplyCurrent());
    SmartDashboard.putNumber("Hood/Voltage", motor.getMotorOutputVoltage());
    SmartDashboard.putBoolean("Hood/GoodToShoot", goodToShoot());
    SmartDashboard.putString("Hood/State", state.name());
  }

  public void createTuningData(){
    SmartDashboard.putBoolean("Hood/Tuning/EnableTuning", false);
    SmartDashboard.putNumber("Hood/Tuning/P", 0);
    SmartDashboard.putNumber("Hood/Tuning/I", 0);
    SmartDashboard.putNumber("Hood/Tuning/D", 0);
    SmartDashboard.putNumber("Hood/Tuning/Goal", 0);
  }

  public boolean tuningEnabled(){
    return SmartDashboard.getBoolean("Hood/Tuning/EnableTuning", false);
  }

  public boolean goodToShoot(){
    return pid.atSetpoint();
  }

  public void stopMotor(){
    motor.set(TalonSRXControlMode.PercentOutput, 0.0);;
  }

  public TalonSRX getMotor(){
    return motor;
  }
}