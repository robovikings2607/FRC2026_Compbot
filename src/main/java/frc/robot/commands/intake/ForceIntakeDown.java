
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ForceIntakeDown extends Command {

  RobotContainer robot;
  IntakeSubsystem intake;
  boolean done = false;

  public ForceIntakeDown(RobotContainer robot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.robot = robot;
    intake = robot.intake;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.forcePivotDown();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.getPivotMotor().getStatorCurrent().getValueAsDouble() > 15.0){
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
          intake.getPivotMotor().setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
