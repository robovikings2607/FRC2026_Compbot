// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.HoodSubsystem.HoodStates;
import frc.robot.utilities.ShooterUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  /** Creates a new TransferPieces. */

  RobotContainer robot;
  FeederSubsystem feeder;
  SpindexerSubsystem spindexer;
  HoodSubsystem hood;
  FlywheelSubsystem flywheel;

  public Shoot(RobotContainer robot) {
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
    flywheel.readyShot(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = robot.drivetrain.getState().Pose;

    Translation2d shooterPose = ShooterUtils.getShooterPose(robotPose);
    Translation2d goalPose = ShooterUtils.determineShootingGoal(robotPose);

    double distance = shooterPose.getDistance(goalPose);

    double rps = flywheel.getGoal(distance);

    if(flywheel.isFixed()){
      flywheel.velocityControl(-50.0); //guessing
      hood.setState(HoodStates.FIXED);
    }
    else if(ShooterUtils.inNeutralZone(robotPose)){
      flywheel.velocityControl(-80.0);
      hood.setState(HoodStates.FERRYING);
    }
    else{
      flywheel.velocityControl(rps);
      hood.setState(HoodStates.SHOOTING);
    }

    hood.controlMotor(distance);

    if(flywheel.goodToShoot() && hood.goodToShoot()){
      feeder.runMotor();
      spindexer.runMotor();
    }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stopMotor();
    spindexer.stopMotor();
    hood.stopMotor();
    flywheel.coastOut();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
