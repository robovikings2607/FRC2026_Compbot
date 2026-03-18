// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.PortUnreachableException;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utilities.ISysIdTunable;
import frc.robot.utilities.SysIdBuilder;
import frc.robot.Constants.FeederConstants;

import static edu.wpi.first.units.Units.*;

public class FeederSubsystem extends SubsystemBase implements ISysIdTunable {
  /** Creates a new FeederSubsystem. */
  //private int reverse;
  private TalonFX feederMotor;
  private VelocityVoltage control = new VelocityVoltage(0.0);
  private double speed;

  public FeederSubsystem(RobotContainer robot) {
    feederMotor = new TalonFX(FeederConstants.FEEDER_ID);
    configureMotor();
    SmartDashboard.putNumber("Feeder/Speed", 0.0);
  }

  private final SysIdRoutine sysIdRoutine = SysIdBuilder.buildTalonFXRoutine(
        feederMotor, this, "feeder", 4.0
  );    

  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    speed = SmartDashboard.getNumber("Feeder/Speed", 0.0);

  }

  public void configureMotor(){
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(85))
            .withStatorCurrentLimitEnable(true)
    );

    var slot0Configs = configs.Slot0;
          // slot0Configs.kS = 0.0; // Voltage output to overcome static friction
          slot0Configs.kV = 0.1075; // A velocity target of 1 rps requires this voltage output.
          // slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires this voltage output
          slot0Configs.kP = 0.5; // A position error of 2.5 rotations requires this voltage output
          slot0Configs.kI = 0; // no output for integrated error
          slot0Configs.kD = 0.000; // A velocity error of 1 rps requires this voltage output

    configs.withSlot0(slot0Configs);

    feederMotor.getConfigurator().apply(configs);
  }

  public void runMotor() {
    //feederMotor.setVoltage(FeederConstants.FEEDER_SPEED);
    feederMotor.setControl(control.withVelocity(75));
  }

  public void stopMotor() {
    feederMotor.setControl(new CoastOut());
  }

  public void reverseMotor() {
    feederMotor.setVoltage(-FeederConstants.FEEDER_SPEED);;
  }
}