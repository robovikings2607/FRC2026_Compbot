// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederSubsystemExp;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.SpindexerSubsystemExp;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UnjamShooterCommandExp extends Command {

  //RobotContainer robot;
  SpindexerSubsystemExp spindexer;
  FeederSubsystemExp feeder;

  public UnjamShooterCommandExp(SpindexerSubsystemExp spindexer, FeederSubsystemExp feeder) {
    this.spindexer = spindexer;
    this.feeder = feeder;

    addRequirements(spindexer, feeder);
  }

  @Override
  public void execute() {
      // Run both systems backward to back the game piece out of the choke point
      spindexer.run(SpindexerConstants.REVERSE_SPEED);
      feeder.run(FeederConstants.REVERSE_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
      spindexer.stop();
      feeder.stop();
  }  
}
