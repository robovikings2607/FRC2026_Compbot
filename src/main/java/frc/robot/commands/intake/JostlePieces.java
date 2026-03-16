// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JostlePieces extends Command {
  /** Creates a new JostlePieces. */

  RobotContainer robot;
  IntakeSubsystem intake;
  Timer timer = new Timer();
  boolean isDeployed = true;

  public JostlePieces(RobotContainer robot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.robot = robot;
    intake = robot.intake;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    intake.forcePivotDown();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isDeployed && timer.get() > 0.25){
      intake.forcePivotUp();
      isDeployed = false;
      timer.stop();
      timer.reset();
      timer.start();
    }

    if(!isDeployed && timer.get() > 0.25){
      intake.forcePivotDown();
      isDeployed = true;
      timer.stop();
      timer.reset();
      timer.start();
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
