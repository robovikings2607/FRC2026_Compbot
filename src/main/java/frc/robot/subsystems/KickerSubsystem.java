// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.KickerConstants;

import static edu.wpi.first.units.Units.*;

public class KickerSubsystem extends SubsystemBase {
  /** Creates a new KickerSubsystem. */

  private RobotContainer robot;

  private TalonFX kickerMotor;

  public KickerSubsystem(RobotContainer robot) {
    this.robot = robot;
  
    configureMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void configureMotor(){
    kickerMotor = new TalonFX(KickerConstants.KICKER_ID);

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.withCurrentLimits(
        new CurrentLimitsConfigs()
            // Swerve azimuth does not require much torque output, so we can set a relatively low
            // stator current limit to help avoid brownouts without impacting performance.
            .withStatorCurrentLimit(Amps.of(20))
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(40))
            .withSupplyCurrentLowerLimit(Amps.of(10))
            .withSupplyCurrentLimitEnable(true)
    );

    kickerMotor.getConfigurator().apply(configs);
  }

  public void runMotor(){
    kickerMotor.setVoltage(KickerConstants.KICKER_SPEED);
  }

  public void stopMotor(){
    kickerMotor.stopMotor();
  }
}
