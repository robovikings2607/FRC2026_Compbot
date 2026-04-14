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
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.KickerSubsystem.KickerState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeployIntakeAndRunKicker extends DeployIntake {
  /** Creates a new RunIntake. */

  RobotContainer robot;
  Timer timer = new Timer();
  boolean hasPulsed = false;

  public DeployIntakeAndRunKicker(RobotContainer robot) {
    super(robot);
    this.robot = robot;
    addRequirements(robot.kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    robot.kicker.controlMotor(KickerState.FORWARD);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    robot.kicker.controlMotor(KickerState.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}