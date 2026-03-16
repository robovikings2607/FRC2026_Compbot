// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.AllPermission;
import java.util.Collections;
import java.util.List;

import org.opencv.core.Mat;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldLocations;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.utilities.GeometryUtil;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.RobotLogger;
import frc.robot.utilities.ShooterUtils;

import static edu.wpi.first.units.Units.*;

public class TurretSubsystem extends SubsystemBase {
  private static final double rotationsPerDegree = 10.0/360.0;
  private final TalonFX turretMotor;
  private final RobotContainer robot;
  private MotionMagicVoltage magicMotionRequest;
  private double previousSetPoint, currentEncoderPos, offset;
  private boolean fixedShot = false;
  private boolean isDeactivated = false;

  public TurretSubsystem(RobotContainer robot) {
    this.robot = robot;

    turretMotor = new TalonFX(TurretConstants.TURRET_ID);

    //turretMotor.setPosition(0);
    //previousSetPoint = turretMotor.getPosition().getValueAsDouble();
        
    // turretMotor.setPosition(0.0);

    configureMotor();
    magicMotionRequest = new MotionMagicVoltage(0.0);

    logNumber2("Turret/BootUpPose", turretMotor.getPosition().getValueAsDouble());
    logNumber2("SetOffset", 0.0);    
    //logNumber2("Turret/MotorCurrent", turretMotor.getStatorCurrent().getValueAsDouble());        
  }

  private void configureMotor() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    var slot0Configs = configs.Slot0;
        slot0Configs.kS = 1.0; // Voltage output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps requires this voltage output.
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires this voltage output
        slot0Configs.kP = 6.0; // A position error of 2.5 rotations requires this voltage output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.15; // A velocity error of 1 rps requires this voltage output

    var motionMagicConfigs = configs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 70; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 2800; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 80000; // Target jerk of 1600 rps/s/s (0.1 seconds)

    //enable software limits
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    //limits (in rotations)
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = rotationsPerDegree * TurretConstants.MAX_ANGLE;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = rotationsPerDegree * TurretConstants.MIN_ANGLE; 
  
    configs.withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );
  
    turretMotor.getConfigurator().apply(configs);
    turretMotor.setNeutralMode(NeutralModeValue.Brake);    
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run

    Pose2d robotPose = robot.drivetrain.getState().Pose;

    double robotRotation = robotPose.getRotation().getDegrees();
    Translation2d shooterPose = ShooterUtils.getShooterPose(robotPose);
    Translation2d goalPose = ShooterUtils.virtualTarget(robot.drivetrain, robotPose);

    //checks alliance and aims at corresponding hub
    double currentEncoderPos = turretMotor.getPosition().getValueAsDouble();
    double targetEncoderPos = getTurretSetPoint(shooterPose, goalPose, robotRotation);

    double wantedEncoderPos = currentEncoderPos + getDelta(currentEncoderPos, targetEncoderPos);

    wantedEncoderPos = clampEncoderPos(wantedEncoderPos);

    offset = SmartDashboard.getNumber("Turret/Offset", 0.0);

    if(isDeactivated){
      turretMotor.setControl(new CoastOut());
    }
    else{
      turretMotor.setControl(magicMotionRequest.withPosition(wantedEncoderPos));
    }

    logNumber2("Turret/Delta", getDelta(currentEncoderPos, targetEncoderPos));        
    logNumber2("Turret/CurrentPose", currentEncoderPos);        
    logNumber2("Turret/TargetPose", targetEncoderPos);        
    logNumber2("Turret/WantedPose", wantedEncoderPos);        
  }

  private static double getTurretSetPoint(Translation2d turretCenter, Translation2d hubCenter, double robotRotation) {
    double angle = GeometryUtil.getTargetAngle(turretCenter, hubCenter);
    double robotRotationAdjustedAngle = MathUtil.inputModulus(angle - robotRotation, -180.0, 180.0);   

    return -robotRotationAdjustedAngle * rotationsPerDegree;
  }

  public static double clampEncoderPos(double newEncoderPos){
/*     if(newEncoderPos > (TurretConstants.MAX_ANGLE * rotationsPerDegree)){
      newEncoderPos -= 360 * rotationsPerDegree;
    }
    else if(newEncoderPos < (TurretConstants.MIN_ANGLE * rotationsPerDegree)){
      newEncoderPos += 360 * rotationsPerDegree;
    } */
    return MathUtil.inputModulus(newEncoderPos, rotationsPerDegree * TurretConstants.MIN_ANGLE, rotationsPerDegree * TurretConstants.MAX_ANGLE);
  }

  public static double getDelta(double previousSetPoint, double newSetPoint){
    double delta = 0;

    if(Math.abs(previousSetPoint - newSetPoint) >= 10.0){ //if turret wraps
      delta = 10 - Math.abs(previousSetPoint - newSetPoint);
     
      if(previousSetPoint < newSetPoint){
        delta = -delta;
      }
    }
    else{
      delta = newSetPoint - previousSetPoint;
    }

    return delta;
  }

  public void fixedShot(boolean fixed){
    fixedShot = fixed;
  }

  public void setTurretToZero(){
    turretMotor.setPosition(0);
  }

/*   public Command zeroTurret(){
    return run(() -> turretMotor.set(0.2))
           .until(() -> isPressed)
           .andThen(() -> turretMotor.set(0.0))
           .andThen(() -> isZeroed())
           .andThen(() -> setTurretToZero());
  } */

  public boolean inTolerance(double pose){
    return turretMotor.getPosition().getValueAsDouble() > pose - 0.5 ||
           turretMotor.getPosition().getValueAsDouble() < pose + 0.5; 
  }

  public void deactivateTurret(boolean deactivate){
    isDeactivated = deactivate;
  }

  public TalonFX getMotor(){
    return turretMotor;
  }

  public void logNumber(String key, double value){
    SmartDashboard.putNumber(key, value);
  }

  public void logNumber2(String key, double value){
    RobotLogger.logDouble(key, value);
  }

}
