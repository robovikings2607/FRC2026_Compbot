// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeployIntake extends Command {
  /** Creates a new RunIntake. */

  RobotContainer robot;
  IntakeSubsystem intake;
  Timer timer = new Timer();

  public DeployIntake(RobotContainer robot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.robot = robot;
    intake = robot.intake;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.forcePivotDown();
    intake.runRollersUnjammed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Check for jam in intake
    if(intake.isJammed()){
      intake.runRollersJammed();
      timer.start();

      if(timer.get() > 1.0){
        intake.reverseRollers();
      }
    }
    else{
      timer.reset();
      intake.runRollersUnjammed();
    }

    //Stop Voltage output and hold current position
    if(intake.currentLimitExceeded()){
      intake.holdDeployPosition(intake.pivotPosition());
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
