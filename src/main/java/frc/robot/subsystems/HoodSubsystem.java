// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utilities.RobotLogger;
import frc.robot.utilities.ShooterUtils;
import frc.robot.Constants.FieldLocations;
import frc.robot.Constants.HoodConstants;
import frc.robot.utilities.GeometryUtil;

import static edu.wpi.first.units.Units.*;


public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */
  private TalonFX hoodMotor;
  private RobotContainer robot;

  private final MotionMagicVoltage magicMotionRequest = new MotionMagicVoltage(0);
  private final CoastOut coastOut = new CoastOut();
  private static final double gearRatio = ((350.0/50.0)*(26.0/12.0));
  private static final double rotationsPerDegree = gearRatio/360.0;
  private double setPoint, goal;
  private InterpolatingDoubleTreeMap hoodInterp = new InterpolatingDoubleTreeMap();
  private boolean readyToShoot = false;
  private boolean fixedShot = false;
  private double targetAngleDegrees = 0.0;    

  // The WPILib Physics Model
// Needs: Gearbox, Gearing Ratio, Moment of Inertia (guess small, like 0.01), 
// Length, Min Angle, Max Angle, Simulate Gravity (usually false for a tight hood)
private final SingleJointedArmSim hoodSim = new SingleJointedArmSim(
    DCMotor.getKrakenX44(1), 
    gearRatio, // Example 100:1 gear ratio
    0.01,  // Moment of Inertia (kg*m^2)
    0.2,   // Length of the hood mechanism (meters)
    Math.toRadians(10), // Minimum angle (Bottomed out)
    Math.toRadians(50), // Maximum angle (Fully extended)
    true,  // Simulate gravity? (usually yes, unless it's a lead screw)
    0.0    // Starting angle
);



  public HoodSubsystem(RobotContainer robot) {
    this.robot = robot;

    configureMotor();
    createInterpMap();
    // zeroMotor();
    SmartDashboard.putNumber("Hood/SetPoint", 0);
  }

  @Override
  public void periodic() {
    RobotLogger.logDouble("Hood/" + "actualAngleDegrees",getActualHoodAngleDegrees());   
    RobotLogger.logDouble("Hood/" + "targetAngleDegrees",targetAngleDegrees);   
  }

  private double getActualHoodAngleDegrees() {
    return GeometryUtil.getMotorRotationsAsDegrees(
      hoodMotor.getPosition().getValueAsDouble(), 
      gearRatio);
  }

  public void configureMotor(){
    hoodMotor = new TalonFX(HoodConstants.HOOD_ID);

    TalonFXConfiguration configs = new TalonFXConfiguration();

    var slot0Configs = configs.Slot0;
        slot0Configs.kS = 1.0; // Voltage output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps requires this voltage output.
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires this voltage output
        slot0Configs.kP = 3.8; // A position error of 2.5 rotations requires this voltage outputp
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.11; // A velocity error of 1 rps requires this voltage output

    var motionMagicConfigs = configs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

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
    hoodInterp.put(3.9, 0.0);
    hoodInterp.put(4.0, -3.0);

    hoodInterp.put(6.0, -3.0);
/*     hoodInterp.put(2.53, 0.0);
    hoodInterp.put(3.1, 0.0);
    hoodInterp.put(3.5, -1.0);
    hoodInterp.put(4.0, -1.0);
    hoodInterp.put(4.5, -2.0);
    hoodInterp.put(4.75, -3.0);
    hoodInterp.put(5.0, -4.0);
    hoodInterp.put(5.125, -4.25);
    hoodInterp.put(5.25, -4.5);
    hoodInterp.put(5.375, -4.75);
    hoodInterp.put(5.5, -5.0);
    hoodInterp.put(5.625, -5.5);
    hoodInterp.put(5.75, -6.0);
    hoodInterp.put(5.875, -6.5);
    hoodInterp.put(6.0, -7.0); */
  }

  public void setGoal(double distance){ 
    targetAngleDegrees  = hoodInterp.get(distance);
    setPoint = targetAngleDegrees * rotationsPerDegree;
  }

  public void positionControl(double angle){
    RobotLogger.logDouble("hood", angle);
    hoodMotor.setControl(magicMotionRequest.withPosition(angle));
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

  public void simulationPeriodic() {
    // 1. Give the WPILib physics model the current voltage from our virtual motor
    hoodSim.setInputVoltage(hoodMotor.getSimState().getMotorVoltage());
    
    // 2. Advance the physics simulation by 20ms
    hoodSim.update(0.020);
    
    // 1. Convert WPILib's radians into friendly degrees
    double hoodDegrees = Units.radiansToDegrees(hoodSim.getAngleRads());

    // 2. Convert degrees to rotations (divide by 360), then multiply by the gear ratio
    hoodMotor.getSimState().setRawRotorPosition(
        (hoodDegrees / 360.0) * gearRatio 
    );

    // 2. Create a fake Pose2d off the bottom of the field.
    // X: 0.0, Y: -1.0 (Off the carpet), Rotation: Your hood angle!
    Pose2d hoodDial = new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(hoodDegrees));
    // 2. Create a static base line that stays permanently locked at 0 degrees (horizontal)
    Pose2d staticBase = new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(0));


    RobotLogger.logStruct("Hood/" + "targetAnglePose", Pose2d.struct, hoodDial);     
    RobotLogger.logStruct("Hood/" + "targetAngleBaseZeroPose", Pose2d.struct, staticBase);     

  }


}