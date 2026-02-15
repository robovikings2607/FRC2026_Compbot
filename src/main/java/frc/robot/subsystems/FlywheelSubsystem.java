// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldElements;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.utilities.GeometryUtil;
import frc.robot.utilities.ShooterUtils;

public class FlywheelSubsystem extends SubsystemBase {
  /** Creates a new ShooterHoodSubsystem. */
  private final TalonFX flywheelMotor;
  private final RobotContainer robot;
  private VelocityDutyCycle velocityControl = new VelocityDutyCycle(0);
  private double rps;
  private InterpolatingDoubleTreeMap flywheelInterp = new InterpolatingDoubleTreeMap();

  public FlywheelSubsystem(RobotContainer robot) {
    this.robot = robot;
    flywheelMotor = new TalonFX(FlywheelConstants.FLYWHEEL_ID);
    configureMotor();
    createInterpMap();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Pose2d robotPose = robot.drivetrain.getState().Pose;

    Translation2d shooterPose = ShooterUtils.getShooterPose(robotPose);
    Translation2d goalPose = new Translation2d();

    if(DriverStation.getAlliance().get() == Alliance.Blue){
      goalPose = FieldElements.BLUE_HUB;      
    }
    else if(DriverStation.getAlliance().get() == Alliance.Red){
      goalPose = FieldElements.RED_HUB;            
    }

    double distance = shooterPose.getDistance(goalPose);
    setGoal(distance);

    flywheelMotor.setControl(velocityControl.withVelocity(rps)
                                            .withAcceleration(100)
                                            .withFeedForward(rps * 0.012));
  }

  public void configureMotor(){ 
    TalonFXConfiguration configs = new TalonFXConfiguration();

    var slot0Configs = configs.Slot0;
          // slot0Configs.kS = 0.0; // Voltage output to overcome static friction
          // slot0Configs.kV = 0.0; // A velocity target of 1 rps requires this voltage output.
          // slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires this voltage output
          slot0Configs.kP = 0.05; // A position error of 2.5 rotations requires this voltage output
          slot0Configs.kI = 0; // no output for integrated error
          slot0Configs.kD = 0.01; // A velocity error of 1 rps requires this voltage output

    flywheelMotor.getConfigurator().apply(configs);
    flywheelMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void createInterpMap(){
    //key = distance from goal
    //value = speed of flywheel in rps 
    flywheelInterp.put(null, null);
  }

  public void setGoal(double distance){
    rps = flywheelInterp.get(distance);
  }

  public double getGoal(){
    return rps;
  }

  public double getSpeed(){
    return flywheelMotor.getVelocity().getValueAsDouble();
  }
}
