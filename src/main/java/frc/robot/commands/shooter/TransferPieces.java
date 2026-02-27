// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TransferPieces extends Command {
  /** Creates a new TransferPieces. */

  RobotContainer robot;
  FeederSubsystem feeder;
  SpindexerSubsystem spindexer;
  HoodSubsystem hood;
  FlywheelSubsystem flywheel;

  public TransferPieces(RobotContainer robot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.robot = robot;
    feeder = robot.feeder;
    spindexer = robot.spindexer;
    hood = robot.hood;
    flywheel = robot.flywheel;

    addRequirements(feeder, spindexer, hood, flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.readyShot(true);
    flywheel.readyShot(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(flywheel.goodToShoot()){
    feeder.runMotor();
    spindexer.runMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stopMotor();
    spindexer.stopMotor();
    hood.readyShot(false);
    flywheel.readyShot(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
