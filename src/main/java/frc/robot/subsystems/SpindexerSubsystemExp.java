// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SpindexerConstants;

public class SpindexerSubsystemExp extends SubsystemBase {
  private TalonFX motor;
  private final DutyCycleOut driveRequest = new DutyCycleOut(0);      

  public SpindexerSubsystemExp(RobotContainer robot) {
    motor = new TalonFX(SpindexerConstants.SPINDEXER_ID);
  }

  @Override
  public void periodic() {

  }

  /**
   * Runs the spindexer at a specific percent speed.
   * @param percent -1.0 to 1.0
   */
  public void run(double percent) {
      motor.setControl(driveRequest.withOutput(percent));
  }

  public void stop() {
      motor.setControl(driveRequest.withOutput(0));
  }

}
