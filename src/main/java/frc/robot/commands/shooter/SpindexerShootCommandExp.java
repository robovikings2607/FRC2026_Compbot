// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystemExp;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.FlywheelSubsystemExp;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.SpindexerSubsystemExp;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpindexerShootCommandExp extends InstantCommand {

  private final SpindexerSubsystemExp spindexer;
  private final FeederSubsystemExp feeder;
  private final BooleanSupplier isReadyToShoot;  

  public SpindexerShootCommandExp(
    SpindexerSubsystemExp spindexer,
    FeederSubsystemExp feeder,
    BooleanSupplier isReadyToShoot) {

    this.feeder = feeder;
    this.spindexer = spindexer;    
    this.isReadyToShoot = isReadyToShoot;

    addRequirements(feeder, spindexer);
  }

  @Override
  public void execute() {

      spindexer.runForwardAtTunedSpeed();

      if (isReadyToShoot.getAsBoolean()) {
          feeder.runForwardAtTunedSpeed(); 
      } else {
          // A ball just went through and the RPM dropped, OR we are still spinning up.
          // STOP the feeder immediately so the next ball waits its turn.
          feeder.stop(); 
      }
  }

  @Override
  public void end(boolean interrupted) {
      spindexer.stop();
      feeder.stop();
  }

}
