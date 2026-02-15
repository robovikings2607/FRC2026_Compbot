// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private int up;
  /** Creates a new ClimberSubsystem. */
  private TalonFX climberMotor;
  public ClimberSubsystem(RobotContainer robot) {
    climberMotor = new TalonFX(ClimberConstants.CLIMBER_ID);
  }
  public void moveMotor() {
    climberMotor.set(.41);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void turnClimberOn(double speed) {}

  public void turnClimberOff() {
    climberMotor.set(0);
  }

  public void upClimber() {
    up = 1;
  }

  public void downClimber() {
    up = -1;
  }
}
