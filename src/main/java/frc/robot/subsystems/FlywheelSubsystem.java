// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldLocations;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.utilities.GeometryUtil;
import frc.robot.utilities.ISysIdTunable;
import frc.robot.utilities.ShooterUtils;
import frc.robot.utilities.SysIdBuilder;

import static edu.wpi.first.units.Units.*;

public class FlywheelSubsystem extends SubsystemBase implements ISysIdTunable {
  /** Creates a new ShooterHoodSubsystem. */
  public final TalonFX flywheelMotor = new TalonFX(FlywheelConstants.FLYWHEEL_ID);
  private final RobotContainer robot;
  private VelocityVoltage velocityControl = new VelocityVoltage(0);
  private CoastOut coastOut = new CoastOut();
  private double rps;
  private InterpolatingDoubleTreeMap flywheelInterp = new InterpolatingDoubleTreeMap();
  private boolean readyToShoot = false;
  private boolean fixedShot = false;

  public FlywheelSubsystem(RobotContainer robot) {
    this.robot = robot;
    configureMotor();
    configureMotorForSysId();
    createInterpMap();

  }

  private final SysIdRoutine sysIdRoutine = SysIdBuilder.buildTalonFXRoutine(
    flywheelMotor, this, "flywheel", 7.0
  );    
 
  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Pose2d robotPose = robot.drivetrain.getState().Pose;

    Translation2d shooterPose = ShooterUtils.getShooterPose(robotPose);
    Translation2d goalPose = ShooterUtils.virtualTarget(robot.drivetrain, robotPose);

    double distance = shooterPose.getDistance(goalPose);
    SmartDashboard.putNumber("Distance", distance);

    SmartDashboard.putNumber("Flywheel/Speed", flywheelMotor.getRotorVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Flywheel/WantedSpeed", getGoal(distance));

    
/*     // rps = SmartDashboard.getNumber("Flywheel/Speed", 0);
    if(!readyToShoot){
      flywheelMotor.setControl(coastOut);
    }
    else{
/*        if(fixedShot){
        flywheelMotor.setControl(velocityControl.withVelocity(50));
      } 
      if(ShooterUtils.inNeutralZone(robotPose)){
        flywheelMotor.setControl(velocityControl.withVelocity(-80.0));
      }
      else{
        setGoal(distance);
        flywheelMotor.setControl(velocityControl.withVelocity(rps));
      }
    }  */
  }

  public void configureMotor(){ 
    TalonFXConfiguration configs = new TalonFXConfiguration();

    var slot0Configs = configs.Slot0;
          // slot0Configs.kS = 0.0; // Voltage output to overcome static friction
          slot0Configs.kV = 0.12; // A velocity target of 1 rps requires this voltage output.
          // slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires this voltage output
          slot0Configs.kP = 0.6; // A position error of 2.5 rotations requires this voltage output
          slot0Configs.kI = 0; // no output for integrated error
          slot0Configs.kD = 0.000; // A velocity error of 1 rps requires this voltage output

    configs.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
        );

    flywheelMotor.setNeutralMode(NeutralModeValue.Coast);
    flywheelMotor.getConfigurator().apply(configs);
  }

    private void configureMotorForSysId() {
    // --- THE SYSID FIX: FORCE HIGH-SPEED DATA LOGGING ---
    // We tell the motor to send Voltage, Position, and Velocity at 250 Hz (every 4 milliseconds)
    flywheelMotor.getMotorVoltage().setUpdateFrequency(250.0);
    flywheelMotor.getPosition().setUpdateFrequency(250.0);
    flywheelMotor.getVelocity().setUpdateFrequency(250.0);
        
    // (Optional but recommended) Wait for the CAN bus to apply the changes
    try { Thread.sleep(250); } catch (InterruptedException e) {}

  }

  public void createInterpMap(){
    //key = distance from goal
    //value = speed of flywheel in rps 
    flywheelInterp.put(0.0, -49.5);
    flywheelInterp.put(2.53, -49.5);
    flywheelInterp.put(3.1, -52.0);
    flywheelInterp.put(3.5, -56.5);
    flywheelInterp.put(4.0, -59.0);
    flywheelInterp.put(4.5, -62.0);
    flywheelInterp.put(5.0, -65.0);
    flywheelInterp.put(5.5, -68.0);
    flywheelInterp.put(6.0, -69.0);
  }

  public void setGoal(double distance){
    rps = flywheelInterp.get(distance);
  }

  public void coastOut(){
    flywheelMotor.setControl(new CoastOut());
  }

  public void velocityControl(double rps){
    flywheelMotor.setControl(velocityControl.withVelocity(rps));
  }

  public double getGoal(double distance){
    rps = flywheelInterp.get(distance);
    return rps;
  }

  public double getSpeed(){
    return flywheelMotor.getVelocity().getValueAsDouble();
  }

  public boolean goodToShoot(){
    return getSpeed() < -45.0;
  }

  public void readyShot(boolean ready){
    readyToShoot = ready;
  }

  public boolean isReady(){
    return readyToShoot;
  }

  public void fixedShot(boolean fixed){
    fixedShot = fixed;
  }

  public void tune(){
    flywheelMotor.setControl(velocityControl.withVelocity(-70));
  }
}
