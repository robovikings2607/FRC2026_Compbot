// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.PortUnreachableException;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SpindexerConstants;

public class SpindexerSubsystem extends SubsystemBase {
  private int reverse;
  /** Creates a new SpinDexerSubSystem. */
 private TalonFX spindexerMotor;
  public SpindexerSubsystem(RobotContainer robot) {
    spindexerMotor = new TalonFX(SpindexerConstants.SPINDEXER_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startMotor(double speed) {
    spindexerMotor.set(SpindexerConstants.SPINDEXER_SPEED);
  }

  public void stopMotor() {
    spindexerMotor.set(0);
  }

  public void reverseMotor() {
    spindexerMotor.set(-SpindexerConstants.SPINDEXER_SPEED);
  }
}
