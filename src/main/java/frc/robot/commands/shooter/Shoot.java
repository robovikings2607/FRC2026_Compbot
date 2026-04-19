// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeederState;
import frc.robot.subsystems.FlywheelSubsystem.FlywheelState;
import frc.robot.subsystems.HoodSubsystem.HoodState;
import frc.robot.subsystems.KickerSubsystem.KickerState;
import frc.robot.subsystems.SpindexerSubsystem.SpindexerState;
import frc.robot.utilities.ShooterUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  /** Creates a new TransferPieces. */

  RobotContainer robot;
  FeederSubsystem feeder;
  SpindexerSubsystem spindexer;
  HoodSubsystem hood;
  FlywheelSubsystem flywheel;
  KickerSubsystem kicker;

  public Shoot(RobotContainer robot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.robot = robot;
    feeder = robot.feeder;
    spindexer = robot.spindexer;
    hood = robot.hood;
    flywheel = robot.flywheel;
    kicker = robot.kicker;

    addRequirements(feeder, hood, flywheel, kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = robot.drivetrain.getState().Pose;

    robot.setLowGear(true);

    Translation2d shooterPose = ShooterUtils.getShooterPose(robotPose);
    Translation2d goalPose = ShooterUtils.stuypulesShootOnMove(robot.drivetrain, robotPose);

    double distance = shooterPose.getDistance(goalPose);

    boolean pidTuningEnabled = SmartDashboard.getBoolean("Tuning/EnablePIDTuning", false);
    boolean distanceTuningEnabled = SmartDashboard.getBoolean("Tuning/EnableDistanceTuning", true);
    /* 
    if(pidTuningEnabled){
      setPIDTuningStates();
    } 
    else if(distanceTuningEnabled){
      setDistanceTuningStates();
    } 
    else */ if(robot.isFixedShot()){ //add boolean check later
      setFixedStates();
    }
    else if(ShooterUtils.inNeutralZone(robotPose)){
      setFerryingStates();
    }
    else{
      setShootingStates();
    }

    controlShooting(distance);

    if(flywheel.goodToShoot() && hood.goodToShoot() && robot.turret.goodToShoot()){
      if(ShooterUtils.inOppoAllianceZone(robotPose)){
        feeder.setState(FeederState.FULL_FIELD);
      if(robot.operatorController.rightBumper.getAsBoolean()){
        //spindexer.setState(SpindexerState.REVERSE);
      }
      else{
        //spindexer.setState(SpindexerState.FORWARD);
    }        //kicker.setState(KickerState.SHOOTING);
      }
      else{
        setFeedingStates();
      }
      kicker.controlMotor(KickerState.SHOOTING);
      controlFeeding();
    }
    else{
      //spindexer.setState(SpindexerState.OFF);
      feeder.setState(FeederState.OFF);
      controlFeeding();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robot.setLowGear(false);
    hood.setState(HoodState.OFF);
    flywheel.setState(FlywheelState.OFF);
    //spindexer.setState(SpindexerState.OFF);
    feeder.setState(FeederState.OFF);
    kicker.controlMotor(KickerState.FORWARD);
    
    controlShooting(0);
    controlFeeding();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void setPIDTuningStates(){
    hood.setState(HoodState.PID_TUNING);
    flywheel.setState(FlywheelState.PID_TUNING);
  }

  public void setDistanceTuningStates(){
    hood.setState(HoodState.DISTANCE_TUNING);
    flywheel.setState(FlywheelState.DISTANCE_TUNING);
  }

  public void setFixedStates(){
    hood.setState(HoodState.FIXED);
    flywheel.setState(FlywheelState.FIXED);
  }

  public void setFerryingStates(){
    hood.setState(HoodState.FERRYING);
    flywheel.setState(FlywheelState.FERRYING);
  }

  public void setShootingStates(){
    hood.setState(HoodState.SHOOTING);
    flywheel.setState(FlywheelState.SHOOTING);
  }

  public void setFeedingStates(){
    feeder.setState(FeederState.FORWARD);
    if(robot.operatorController.rightBumper.getAsBoolean()){
      //spindexer.setState(SpindexerState.REVERSE);
    }
    else{
      //spindexer.setState(SpindexerState.FORWARD);
    }
  }

  public void controlShooting(double distance){
    hood.controlMotor(distance);
    flywheel.controlMotor(distance);
  }

  public void controlFeeding(){
    feeder.controlMotor();
   // spindexer.controlMotor();
    //kicker.controlMotor();
  }
}
