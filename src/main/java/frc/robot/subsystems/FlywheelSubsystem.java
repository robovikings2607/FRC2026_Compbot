// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.utilities.ISysIdTunable;
import frc.robot.utilities.RobotLogger;
import frc.robot.utilities.SysIdBuilder;

import static edu.wpi.first.units.Units.*;

public class FlywheelSubsystem extends SubsystemBase implements ISysIdTunable {
  /** Creates a new FlywheelSubsystem. */
  private RobotContainer robot;
  private final TalonFX motor = new TalonFX(FlywheelConstants.MOTOR_ID);
  private final TalonFXConfiguration configs = new TalonFXConfiguration();
  private final VelocityVoltage pid = new VelocityVoltage(0.0);
  private final CoastOut coastOut = new CoastOut();
  private final InterpolatingDoubleTreeMap shootingInterp = new InterpolatingDoubleTreeMap(); 
  private final InterpolatingDoubleTreeMap ferryingInterp = new InterpolatingDoubleTreeMap();
  private FlywheelState state = FlywheelState.SHOOTING;
  private double goal;

  public FlywheelSubsystem(RobotContainer robot) {
    this.robot = robot;

    configureMotor();
    //configureControlModes();
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
    Slot0Configs slot0Configs = configs.Slot0;
          slot0Configs.kS = FlywheelConstants.S; // Voltage output to overcome static friction
          slot0Configs.kV = FlywheelConstants.V; // A velocity target of 1 rps requires this voltage output.
          slot0Configs.kP = FlywheelConstants.P; // A position error of 2.5 rotations requires this voltage output
          slot0Configs.kI = FlywheelConstants.I; // no output for integrated error
          slot0Configs.kD = FlywheelConstants.D; // A velocity error of 1 rps requires this voltage output

    configs.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(FlywheelConstants.STATOR_LIMIT))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(FlywheelConstants.SUPPLY_LIMIT))
                .withSupplyCurrentLowerLimit(Amps.of(FlywheelConstants.SUPPLY_LOWER_LIMIT))
                .withSupplyCurrentLimitEnable(true)
        );

    configs.withSlot0(slot0Configs);

    motor.getConfigurator().apply(configs);
  }

  public void configureControlModes(){}

  /*
  * Use this method and comment out the normal configureMotor method when running SysId tests
  * on the motor to determine the optimal Ks, Kv, and Ka values for the configureMotor method.
  * The high speed logging is necessary. If you don't do it you'll get an error in the SysId
  * tool when you load the data.
  */
  public void configureMotorForSysId() {
    // --- THE SYSID FIX: FORCE HIGH-SPEED DATA LOGGING ---
    // We tell the motor to send Voltage, Position, and Velocity at 250 Hz (every 4 milliseconds)
    motor.getMotorVoltage().setUpdateFrequency(250.0);
    motor.getPosition().setUpdateFrequency(250.0);
    motor.getVelocity().setUpdateFrequency(250.0);
        
    // (Optional but recommended) Wait for the CAN bus to apply the changes
    try { Thread.sleep(250); } catch (InterruptedException e) {}
  }

  public enum FlywheelState{
    SHOOTING,
    FERRYING,
    FIXED,
    PID_TUNING,
    DISTANCE_TUNING,
    OFF
  }

  public void setState(FlywheelState state){
    this.state = state;
  }

  public FlywheelState getState(){
    return state;
  }

  public void createShootingInterpMap(){
    //key = distance from goal
    //value = speed of flywheel in rps 
    shootingInterp.put(0.0, -40.0);
    shootingInterp.put(1.5, -40.0);
    shootingInterp.put(2.0, -42.5);
    shootingInterp.put(2.5, -45.0);
    shootingInterp.put(3.0, -47.5);
    shootingInterp.put(3.5, -50.0);
    shootingInterp.put(4.0, -52.5);
    shootingInterp.put(4.5, -54.0);
    shootingInterp.put(5.0, -56.5);
    shootingInterp.put(5.5, -61.0);
    shootingInterp.put(5.8, -61.0);
    //shootingInterp.put(6.0, 0.0);
  }

  public void createFerryingInterpMap(){
    //key = distance from goal
    //value = speed of flywheel in rps
    ferryingInterp.put(0.0, -37.0);
    ferryingInterp.put(3.5, -37.0);
    ferryingInterp.put(4.0, -40.0);
    ferryingInterp.put(4.5, -43.0);
    ferryingInterp.put(5.0, -46.0);
    ferryingInterp.put(5.5, -49.0);
    ferryingInterp.put(6.0, -52.0);
    ferryingInterp.put(6.5, -55.0);
    ferryingInterp.put(7.0, -58.0);
    ferryingInterp.put(8.0, -67.0);
    //over bump
   /*  ferryingInterp.put(10.5, -45.0);
    ferryingInterp.put(11.0, -49.0);
    ferryingInterp.put(11.5, -51.0);
    ferryingInterp.put(12.5, -57.0);
    ferryingInterp.put(13.0, -60.0); */
    ferryingInterp.put(13.0, -67.0);

    //ferryingInterp.put(6.0, 0.0);
  }

  public double getGoal(){
    return goal;
  }

  public double getSpeed(){
    return motor.getVelocity().getValueAsDouble();
  }

  public void shootingControl(double distance){
    goal = shootingInterp.get(distance);
    motor.setControl(pid.withVelocity(goal));
  }

  public void ferryingControl(double distance){
    goal = ferryingInterp.get(distance);
    motor.setControl(pid.withVelocity(goal));
  }

  public void fixedControl(){
    goal = shootingInterp.get(3.0);
    motor.setControl(pid.withVelocity(goal));
  }

  public void PIDTuningControl(){
    configs.Slot0.kS = SmartDashboard.getNumber("Flywheel/Tuning/PID/S", 0);
    configs.Slot0.kV = SmartDashboard.getNumber("Flywheel/Tuning/PID/V", 0);
    configs.Slot0.kP = SmartDashboard.getNumber("Flywheel/Tuning/PID/P", 0);
    configs.Slot0.kI = SmartDashboard.getNumber("Flywheel/Tuning/PID/I", 0);
    configs.Slot0.kD = SmartDashboard.getNumber("Flywheel/Tuning/PID/D", 0);
    goal = SmartDashboard.getNumber("Flywheel/Tuning/Goal(RPS)", 0.0);
    motor.setControl(pid.withVelocity(goal));
  }

  public void distanceTuningControl(){
    goal = SmartDashboard.getNumber("Flywheel/Tuning/Goal(RPS)", 0.0);
    motor.setControl(pid.withVelocity(goal));
  }

  public void coastOut(){
    motor.setControl(coastOut);
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
        distanceTuningControl();
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
    RobotLogger.logDouble("Flywheel/Goal(RPS)", goal);
    RobotLogger.logDouble("Flywheel/CurrentRPS", motor.getVelocity().getValueAsDouble());
    RobotLogger.logDouble("Flywheel/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    RobotLogger.logDouble("Flywheel/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    RobotLogger.logDouble("Flywheel/Voltage", motor.getMotorVoltage().getValueAsDouble());
    RobotLogger.logBoolean("Flywheel/GoodToShoot", goodToShoot());
    RobotLogger.logString("Flywheel/State", state.name());
  }

  public void createTuningData(){
    RobotLogger.logDouble("Flywheel/Tuning/PID/S", 0);
    RobotLogger.logDouble("Flywheel/Tuning/PID/V", 0);
    RobotLogger.logDouble("Flywheel/Tuning/PID/P", 0);
    RobotLogger.logDouble("Flywheel/Tuning/PID/I", 0);
    RobotLogger.logDouble("Flywheel/Tuning/PID/D", 0);
    RobotLogger.logDouble("Flywheel/Tuning/Goal(RPS)", 0);
  }  

  public boolean goodToShoot(){
    return getSpeed() < goal + 2.5;
  }

  public double percentOfGoal(){
    return motor.getVelocity().getValueAsDouble() / goal;
  }
  
  private final SysIdRoutine sysIdRoutine = SysIdBuilder.buildTalonFXRoutine(
    motor, this, "flywheel", 7.0
  );    
 
  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  } 
  
  public TalonFX getMotor(){
    return motor;
  }
}
