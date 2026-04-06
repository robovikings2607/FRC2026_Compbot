// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.KickerSubsystem.KickerState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetractIntake extends Command {
  
  RobotContainer robot;
  IntakeSubsystem intake;
  KickerSubsystem kicker;

  public RetractIntake(RobotContainer robot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.robot = robot;
    intake = robot.intake;
    kicker = robot.kicker;

    addRequirements(intake, kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kicker.controlMotor(KickerState.OFF);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.controlIntake(IntakeState.RETRACTED); //needs to be in execute to calculate
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}