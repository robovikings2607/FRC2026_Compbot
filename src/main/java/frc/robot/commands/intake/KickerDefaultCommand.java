// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.KickerSubsystem.KickerState;
import frc.robot.subsystems.SpindexerSubsystem.SpindexerState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class KickerDefaultCommand extends Command {
  /** Creates a new KickerDefaultCommand. */

  RobotContainer robot;
  KickerSubsystem kicker;

  public KickerDefaultCommand(RobotContainer robot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.robot = robot;
    kicker = robot.kicker;

    addRequirements(kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(robot.spindexer.getState().equals(SpindexerState.REVERSE)){
      kicker.controlMotor(KickerState.REVERSE);
    }
    else if(robot.intake.getState().equals(IntakeState.DEPLOYED)){
      kicker.controlMotor(KickerState.FORWARD);
    }
    else if(robot.intake.getState().equals(IntakeState.RETRACTED)){
      kicker.controlMotor(KickerState.OFF);
    }
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
