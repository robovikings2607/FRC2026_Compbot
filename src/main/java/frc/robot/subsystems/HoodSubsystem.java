// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utilities.RobotLogger;
import frc.robot.utilities.ShooterUtils;
import frc.robot.Constants.FieldLocations;
import frc.robot.Constants.HoodConstants;

import static edu.wpi.first.units.Units.*;


public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */
  private TalonSRX hoodMotor;
  private CANcoder encoder;
  private RobotContainer robot;

  private PhoenixPIDController pid = new PhoenixPIDController(7.0, 0.0, 0.0);

  private final PositionVoltage control = new PositionVoltage(0);
  private final CoastOut coastOut = new CoastOut();
  private static final double gearRatio = ((350.0/50.0)*(26.0/12.0));
  private static final double rotationsPerDegree = gearRatio/360.0;
  private double setPoint = 0;
  private InterpolatingDoubleTreeMap hoodInterp = new InterpolatingDoubleTreeMap();
  private boolean readyToShoot = false;
  private boolean fixedShot = false;
  private boolean tuning = true;

  public HoodSubsystem(RobotContainer robot) {
    this.robot = robot;

    configureMotor();
    configureEncoder();
    createInterpMap();
    // zeroMotor();

    RobotLogger.logDouble("Hood/SetPoint", 0);                            
    RobotLogger.logBoolean("Hood/Tuning/EnableTuning", tuning);                            
    RobotLogger.logDouble("Hood/Tuning/P", 0);                            
    RobotLogger.logDouble("Hood/Tuning/I", 0);                            
    RobotLogger.logDouble("Hood/Tuning/D", 0);                            
    RobotLogger.logDouble("Hood/Tuning/Goal", 0);                            

    //hoodMotor.setControl(control.withPosition(HoodConstants.MAX_HOOD_POSITION/2));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
/*     if(!readyToShoot){
      hoodMotor.setControl(coastOut);
    }
    else{
 /*      if(fixedShot){
        hoodMotor.setControl(magicMotionRequest.withPosition(0));
      }
      if(ShooterUtils.inNeutralZone(robotPose)){
        hoodMotor.setControl(magicMotionRequest.withPosition(HoodConstants.MAX_HOOD_POSITION));
      }
      else{
        setGoal(distance);
        hoodMotor.setControl(magicMotionRequest.withPosition(setPoint));
      }
    } */

    //setPoint = SmartDashboard.getNumber("Hood/SetPoint", 0) * rotationsPerDegree;
    // hoodMotor.setControl(magicMotionRequest.withPosition(setPoint));

   /*  if(RobotController.getUserButton()){
      hoodMotor.setPosition(0.0);
    } */

    if(tuning){
      pid.setP(SmartDashboard.getNumber("Hood/Tuning/P", 0));
      pid.setI(SmartDashboard.getNumber("Hood/Tuning/I", 0));
      pid.setD(SmartDashboard.getNumber("Hood/Tuning/D", 0));
      setPoint = SmartDashboard.getNumber("Hood/Tuning/Goal", 0);
    }

    double output = -pid.calculate(encoder.getAbsolutePosition().getValueAsDouble(), setPoint, Timer.getFPGATimestamp());
    RobotLogger.logDouble("Hood/Output", output);                                
    hoodMotor.set(TalonSRXControlMode.PercentOutput, output);
  }


  public void configureMotor(){
    hoodMotor = new TalonSRX(HoodConstants.HOOD_ID);  
    TalonSRXConfiguration configs = new TalonSRXConfiguration();
  }

  public void configureEncoder(){
    encoder = new CANcoder(HoodConstants.ENCODER_ID);
    CANcoderConfiguration configs = new CANcoderConfiguration();

    configs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.0;
    configs.MagnetSensor.MagnetOffset = HoodConstants.ENCODER_MAGNET_OFFSET;

    encoder.getConfigurator().apply(configs);
  }

  public void createInterpMap(){
    //key = distance from goal
    //value = position of hood in encoder values
    hoodInterp.put(0.0, 0.0);
    hoodInterp.put(6.0, 0.0);
    //hoodInterp.put(4.5, 0.0);
    //hoodInterp.put(5.0, -2.0);
    //hoodInterp.put(5.6, -4.0);
    //hoodInterp.put(6.0, -6.0);
  }

  public void setGoal(double distance){ 
    setPoint = hoodInterp.get(distance);
  }

  
  public double getGoal(){
    return setPoint;
  }

  public void readyShot(boolean ready){
    readyToShoot = ready;
  }

  public void fixedShot(boolean fixed){
    fixedShot = fixed;
  }

  public TalonSRX getMotor(){
    return hoodMotor;
  }

   /*public void positionControl(double angle){
    RobotLogger.logDouble("hood", angle);
    hoodMotor.setControl(control.withPosition(angle));
    //hoodMotor2.set(TalonSRXControlMode.Position, angle);
  }

  public void coastOut(){
    hoodMotor.setControl(new CoastOut());
  }

  public void zeroMotor(){
    hoodMotor.set(0.2);

    if(hoodMotor.getStatorCurrent().getValueAsDouble() > 25){
      hoodMotor.set(0);
      hoodMotor.setPosition(HoodConstants.MIN_HOOD_ANGLE * rotationsPerDegree);
    }
  } */
/* 
  public double getGoal(double distance){
    return hoodInterp.get(distance) * rotationsPerDegree;
    //return setPoint * rotationsPerDegree;
  } */
}