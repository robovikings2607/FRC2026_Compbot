// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.PortUnreachableException;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utilities.ISysIdTunable;
import frc.robot.utilities.SysIdBuilder;
import frc.robot.Constants.SpindexerConstants;

import static edu.wpi.first.units.Units.*;

public class SpindexerSubsystem extends SubsystemBase implements ISysIdTunable {
  /** Creates a new SpinDexerSubSystem. */
  private TalonFX motor;

  public SpindexerSubsystem(RobotContainer robot) {
    configureMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void configureMotor(){
    motor = new TalonFX(SpindexerConstants.SPINDEXER_ID);
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.withCurrentLimits(
        new CurrentLimitsConfigs()
            // Swerve azimuth does not require much torque output, so we can set a relatively low
            // stator current limit to help avoid brownouts without impacting performance.
            .withStatorCurrentLimit(Amps.of(120))
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(40))
            .withSupplyCurrentLowerLimit(Amps.of(10))
            .withSupplyCurrentLimitEnable(true)
    );

    motor.getConfigurator().apply(configs);
  }

  public void runMotor() {
    motor.setVoltage(SpindexerConstants.SPINDEXER_SPEED);
  }

  public void stopMotor() {
    motor.stopMotor();
  }

  public void reverseMotor() {
    motor.setVoltage(-SpindexerConstants.SPINDEXER_SPEED);
  }

  private final SysIdRoutine sysIdRoutine = SysIdBuilder.buildTalonFXRoutine(
      motor, this, "spindexer", 4.5
  );    

   public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }    

}
