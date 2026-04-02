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
  private HoodState state = HoodState.SHOOTING;
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

  public enum HoodState{
    SHOOTING,
    FERRYING,
    FIXED,
    PID_TUNING,
    DISTANCE_TUNING
  }

  public void setState(HoodState state){
    this.state = state;
  }

  public HoodState getState(){
    return state;
  }

  public void createShootingInterpMap(){
    shootingInterp = new InterpolatingDoubleTreeMap();
    //key = distance from goal
    //value = position of hood in encoder values
    shootingInterp.put(0.0, 0.0);
    shootingInterp.put(6.0, 0.0);
  }
  public void createFerryingInterpMap(){
    ferryingInterp = new InterpolatingDoubleTreeMap();
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

      case DISTANCE_TUNING:
        disatnceTuningControl();
    
      default:
        shootingControl(distance);
        break;
    }
  }

  public void updateLoggingData(){
    SmartDashboard.putNumber("Hood/Output", output);
    SmartDashboard.putNumber("Hood/Goal(Degrees)", goal);
    SmartDashboard.putNumber("Hood/StatorCurrent", motor.getStatorCurrent());
    SmartDashboard.putNumber("Hood/SupplyCurrent", motor.getSupplyCurrent());
    SmartDashboard.putNumber("Hood/Voltage", motor.getMotorOutputVoltage());
    SmartDashboard.putBoolean("Hood/GoodToShoot", goodToShoot());
    SmartDashboard.putString("Hood/State", state.name());
  }

  public void createTuningData(){
    SmartDashboard.putNumber("Hood/Tuning/PID/P", 0);
    SmartDashboard.putNumber("Hood/Tuning/PID/I", 0);
    SmartDashboard.putNumber("Hood/Tuning/PID/D", 0);
    SmartDashboard.putNumber("Hood/Tuning/Goal(Degrees)", 0);
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