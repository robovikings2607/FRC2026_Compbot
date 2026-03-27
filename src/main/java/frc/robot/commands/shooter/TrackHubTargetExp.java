package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldLocations;
import frc.robot.subsystems.TurretSubsystemExp;
import frc.robot.utilities.AimingCalculator;
import frc.robot.utilities.AimingSolution;

public class TrackHubTargetExp extends Command {

  private final TurretSubsystemExp turret;
  private final Supplier<Pose2d> robotPoseProvider;  
  private final Supplier<ChassisSpeeds> robotVelocityProvider;    
  private final AimingCalculator aimingMap = new AimingCalculator();  


  public TrackHubTargetExp(
    TurretSubsystemExp turret, 
    Supplier<Pose2d> robotPoseProvider,
    Supplier<ChassisSpeeds> robotVelocityProvider
  ) {
    this.turret = turret;
    this.robotPoseProvider = robotPoseProvider;    
    this.robotVelocityProvider = robotVelocityProvider;        
    
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d robotPose = robotPoseProvider.get();
    ChassisSpeeds robotVelocity = robotVelocityProvider.get();

    if (DriverStation.getAlliance().isPresent()) {
      Translation2d hubCoordinates = DriverStation.getAlliance().get() == Alliance.Blue ? 
        FieldLocations.BLUE_HUB : FieldLocations.RED_HUB;
        
        
            Pose2d turretPose = turret.getTurretPose(robotPose); 

            AimingSolution solution = aimingMap.calculateMovingAimingSolution(
                turretPose, 
                hubCoordinates, 
                robotVelocity
            );

            turret.trackTarget(solution.virtualTarget(), robotPose);
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