// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.utilities.RobotLogger;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */
  private RobotContainer robot;
  private final TalonSRX motor = new TalonSRX(HoodConstants.MOTOR_ID);
  private final CANcoder encoder = new CANcoder(HoodConstants.ENCODER_ID);
  private final PhoenixPIDController pid = new PhoenixPIDController(HoodConstants.P, HoodConstants.I, HoodConstants.D);
  private final InterpolatingDoubleTreeMap shootingInterp = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap ferryingInterp = new InterpolatingDoubleTreeMap();
  private HoodState state = HoodState.OFF;
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
    SupplyCurrentLimitConfiguration supply = new SupplyCurrentLimitConfiguration();
    supply.currentLimit = HoodConstants.SUPPLY_LIMIT;

    motor.configPeakCurrentLimit(HoodConstants.PEAK_LIMIT);
    motor.configContinuousCurrentLimit(HoodConstants.CONTINUOUS_LIMIT);
    motor.configSupplyCurrentLimit(supply);
  }

  public void configureEncoder(){
    CANcoderConfiguration configs = new CANcoderConfiguration();

    //ensures no jump discontinuity occurs
    configs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = HoodConstants.DISCONTINUITY_POINT;
    configs.MagnetSensor.MagnetOffset = HoodConstants.MAGNET_OFFSET;

    encoder.getConfigurator().apply(configs);
  }

  public void configurePID(){
    pid.setTolerance(HoodConstants.TOLERANCE);
  }

  public enum HoodState{
    SHOOTING,
    FERRYING,
    FIXED,
    PID_TUNING,
    DISTANCE_TUNING,
    OFF
  }

  public void setState(HoodState state){
    this.state = state;
  }

  public HoodState getState(){
    return state;
  }

  public void createShootingInterpMap(){
    //key = distance from goal
    //value = position of hood in encoder values
    shootingInterp.put(0.0, 0.0);
    shootingInterp.put(1.5, 0.0);
    shootingInterp.put(2.0, -1.0);
    shootingInterp.put(2.5, -2.0);
    shootingInterp.put(3.0, -4.0);
    shootingInterp.put(3.5, -6.0);
    shootingInterp.put(4.0, -9.0);
    shootingInterp.put(4.5, -12.0);
    shootingInterp.put(5.0, -14.0);
    //shootingInterp.put(5.5, 0.0);
    //shootingInterp.put(6.0, 0.0);
  }

  public void createFerryingInterpMap(){
    //key = distance from goal
    //value = position of hood in degrees
    ferryingInterp.put(0.0, 0.0);
    ferryingInterp.put(6.0, 0.0);
  }

  public double degreesToEncoderTick(double degrees){
    return degrees/360 * HoodConstants.GEAR_RATIO;
  }

  public double encoderTicksToDegrees(double ticks){
    return ticks/HoodConstants.GEAR_RATIO * 360;
  }

  public void setGoalFromInterp(InterpolatingDoubleTreeMap interp, double distance){ 
    goal = interp.get(distance);
    MathUtil.clamp(goal, HoodConstants.MAX_ANGLE, HoodConstants.MIN_ANGLE);
    goal = degreesToEncoderTick(goal);
  }

  public void setGoalInDegrees(double degrees){
    goal = degreesToEncoderTick(degrees);
  }

  public void setGoalInTicks(double ticks){
    goal = ticks;
  }

  public double getGoal(){
    return goal;
  }

  public void shootingControl(double distance){
    setGoalFromInterp(shootingInterp, distance);
    output = -pid.calculate(encoder.getAbsolutePosition().getValueAsDouble(), goal, Timer.getFPGATimestamp());
    motor.set(TalonSRXControlMode.PercentOutput, output);
  }  
  
  public void ferryingControl(double distance){
    setGoalFromInterp(ferryingInterp, distance);
    output = -pid.calculate(encoder.getAbsolutePosition().getValueAsDouble(), goal, Timer.getFPGATimestamp());
    motor.set(TalonSRXControlMode.PercentOutput, output);
  }  
  
  public void fixedControl(){
    setGoalFromInterp(shootingInterp, 3.0);
    output = -pid.calculate(encoder.getAbsolutePosition().getValueAsDouble(), goal, Timer.getFPGATimestamp());
    motor.set(TalonSRXControlMode.PercentOutput, output);
  }

  public void PIDTuningControl(){
    pid.setP(SmartDashboard.getNumber("Hood/Tuning/PID/P", 0));
    pid.setI(SmartDashboard.getNumber("Hood/Tuning/PID/I", 0));
    pid.setD(SmartDashboard.getNumber("Hood/Tuning/PID/D", 0));
    setGoalInDegrees(SmartDashboard.getNumber("Hood/Tuning/Goal(Degrees)", 0));
    output = -pid.calculate(encoder.getAbsolutePosition().getValueAsDouble(), goal, Timer.getFPGATimestamp());
    motor.set(TalonSRXControlMode.PercentOutput, output);
  }

  public void disatnceTuningControl(){
    setGoalInDegrees(SmartDashboard.getNumber("Hood/Tuning/Goal(Degrees)", 0));
    output = -pid.calculate(encoder.getAbsolutePosition().getValueAsDouble(), goal, Timer.getFPGATimestamp());
    motor.set(TalonSRXControlMode.PercentOutput, output);
  }

  public void stopMotor(){
    motor.set(TalonSRXControlMode.PercentOutput, 0.0);;
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

      case PID_TUNING:
        PIDTuningControl();
        break;

      case DISTANCE_TUNING:
        disatnceTuningControl();
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
    RobotLogger.logDouble("Hood/Output", output);
    RobotLogger.logDouble("Hood/Goal(Degrees)", goal);
    RobotLogger.logDouble("Hood/CurrentDegrees", encoderTicksToDegrees(encoder.getAbsolutePosition().getValueAsDouble()));
    RobotLogger.logDouble("Hood/CurrentPosition", encoder.getAbsolutePosition().getValueAsDouble());
    RobotLogger.logDouble("Hood/StatorCurrent", motor.getStatorCurrent());
    RobotLogger.logDouble("Hood/SupplyCurrent", motor.getSupplyCurrent());
    RobotLogger.logDouble("Hood/Voltage", motor.getMotorOutputVoltage());
    RobotLogger.logBoolean("Hood/GoodToShoot", goodToShoot());
    RobotLogger.logString("Hood/State", state.name());
  }

  public void createTuningData(){
    RobotLogger.logDouble("Hood/Tuning/PID/P", 0);
    RobotLogger.logDouble("Hood/Tuning/PID/I", 0);
    RobotLogger.logDouble("Hood/Tuning/PID/D", 0);
    RobotLogger.logDouble("Hood/Tuning/Goal(Degrees)", 0);
  }

  public boolean goodToShoot(){
    return pid.atSetpoint();
  }

  public TalonSRX getMotor(){
    return motor;
  }

  public CANcoder getEncoder(){
    return encoder;
  }
}