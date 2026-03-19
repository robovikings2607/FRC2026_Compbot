// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
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
  private TalonFX hoodMotor;
  private RobotContainer robot;

  private final PositionVoltage control = new PositionVoltage(0);
  private final CoastOut coastOut = new CoastOut();
  private static final double gearRatio = ((350.0/50.0)*(26.0/12.0));
  private static final double rotationsPerDegree = gearRatio/360.0;
  private double setPoint, goal;
  private InterpolatingDoubleTreeMap hoodInterp = new InterpolatingDoubleTreeMap();
  private boolean readyToShoot = false;
  private boolean fixedShot = false;


  public HoodSubsystem(RobotContainer robot) {
    this.robot = robot;

    configureMotor();
    createInterpMap();
    // zeroMotor();
    SmartDashboard.putNumber("Hood/SetPoint", 0);
    hoodMotor.setControl(control.withPosition(HoodConstants.MAX_HOOD_POSITION/2));
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

    SmartDashboard.putNumber("Hood/Positioj", hoodMotor.getPosition().getValueAsDouble());
  }


  public void configureMotor(){
    hoodMotor = new TalonFX(HoodConstants.HOOD_ID);

    TalonFXConfiguration configs = new TalonFXConfiguration();

    var slot0Configs = configs.Slot0;
  /*       slot0Configs.kS = 1.0; // Voltage output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps requires this voltage output.
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires this voltage output */
        slot0Configs.kP = 7.2; // A position error of 2.5 rotations requires this voltage outputp
        slot0Configs.kI = 0.0; // no output for integrated error
        slot0Configs.kD = 0.0; // A velocity error of 1 rps requires this voltage output

  /*   var motionMagicConfigs = configs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds) */

     //enable software limits
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    //limits (in rotations)
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HoodConstants.MIN_HOOD_ANGLE * rotationsPerDegree;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HoodConstants.MAX_HOOD_ANGLE * rotationsPerDegree; 
  
    configs.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );

    hoodMotor.getConfigurator().apply(configs);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);

    //hoodMotor.setPosition(0);
  }

  public void createInterpMap(){
    //key = distance from goal
    //value = position of hood in desired shot angle
    hoodInterp.put(0.0, 0.0);
    hoodInterp.put(6.0, 0.0);
    //hoodInterp.put(4.5, 0.0);
    //hoodInterp.put(5.0, -2.0);
    //hoodInterp.put(5.6, -4.0);
    //hoodInterp.put(6.0, -6.0);
  }

  public void setGoal(double distance){ 
    setPoint = hoodInterp.get(distance) * rotationsPerDegree;
  }

  public void positionControl(double angle){
    RobotLogger.logDouble("hood", angle);
    hoodMotor.setControl(control.withPosition(angle));
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
  }

  public double getGoal(double distance){
    return hoodInterp.get(distance) * rotationsPerDegree;
    //return setPoint * rotationsPerDegree;
  }

  public double getSetPoint(){
    return setPoint;
  }

  public void readyShot(boolean ready){
    readyToShoot = ready;
  }

  public void fixedShot(boolean fixed){
    fixedShot = fixed;
  }

  public TalonFX getMotor(){
    return hoodMotor;
  }
}