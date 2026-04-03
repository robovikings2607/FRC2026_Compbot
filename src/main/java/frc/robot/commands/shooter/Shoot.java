// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.FlywheelSubsystem.FlywheelState;
import frc.robot.subsystems.HoodSubsystem.HoodState;
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

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = robot.drivetrain.getState().Pose;

    Translation2d shooterPose = ShooterUtils.getShooterPose(robotPose);
    Translation2d goalPose = ShooterUtils.determineShootingGoal(robotPose);

    double distance = shooterPose.getDistance(goalPose);

    boolean pidTuningEnabled = SmartDashboard.getBoolean("Tuning/EnablePIDTuning", false);
    boolean distanceTuningEnabled = SmartDashboard.getBoolean("Tuning/EnableDistanceTuning", false);

    if(pidTuningEnabled){
      hood.setState(HoodState.PID_TUNING);
      flywheel.setState(FlywheelState.PID_TUNING);
    }
    else if(distanceTuningEnabled){
      hood.setState(HoodState.DISTANCE_TUNING);
      flywheel.setState(FlywheelState.DISTANCE_TUNING);
    }
    else if(false){ //add boolean check later
      hood.setState(HoodState.FIXED);
      flywheel.setState(FlywheelState.FIXED);
    }
    else if(ShooterUtils.inNeutralZone(robotPose)){
      hood.setState(HoodState.FERRYING);
      flywheel.setState(FlywheelState.FERRYING);
    }
    else{
      hood.setState(HoodState.SHOOTING);
      flywheel.setState(FlywheelState.SHOOTING);
    }

    flywheel.controlMotor(distance);
    hood.controlMotor(distance);

    if(pidTuningEnabled){
      return;
    }

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
    hood.setState(HoodState.OFF);
    flywheel.setState(FlywheelState.OFF);
    
    hood.controlMotor(0);
    flywheel.controlMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
