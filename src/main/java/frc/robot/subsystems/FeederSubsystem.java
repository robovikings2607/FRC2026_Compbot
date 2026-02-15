// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.PortUnreachableException;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */
  //private int reverse;
  private TalonFX feederMotor;

  public FeederSubsystem(RobotContainer robot) {
    feederMotor = new TalonFX(FeederConstants.FEEDER_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startMotor() {
    feederMotor.set(FeederConstants.FEEDER_SPEED);
  }

  public void stopMotor() {
    feederMotor.set(0);
  }

  public void reverseMotor() {
    feederMotor.set(-FeederConstants.FEEDER_SPEED);;
  }
}