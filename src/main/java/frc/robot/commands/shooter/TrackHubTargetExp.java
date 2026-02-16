package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldElements;
import frc.robot.subsystems.TurretSubsystemExp;

public class TrackHubTargetExp extends Command {

  private final TurretSubsystemExp turretSubsystem;
  private final Supplier<Pose2d> robotPoseProvider;  
  private final Supplier<ChassisSpeeds> robotVelocityProvider;    

  public TrackHubTargetExp(
    TurretSubsystemExp turretSubsystem, 
    Supplier<Pose2d> robotPoseProvider,
    Supplier<ChassisSpeeds> robotVelocityProvider
  ) {
    this.turretSubsystem = turretSubsystem;
    this.robotPoseProvider = robotPoseProvider;    
    this.robotVelocityProvider = robotVelocityProvider;        
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (DriverStation.getAlliance().isPresent()) {
      Translation2d hubCoordinates = DriverStation.getAlliance().get() == Alliance.Blue ? 
        FieldElements.BLUE_HUB : FieldElements.RED_HUB;
        
        
        turretSubsystem.trackTarget(robotPoseProvider.get(), hubCoordinates, robotVelocityProvider.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}