package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldElements;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystemExp;
import frc.robot.subsystems.FlywheelSubsystemExp;
import frc.robot.subsystems.HoodSubsystemExp;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystemExp;

public class AutoAimAndShootCommandExp extends Command {
    private final TurretSubsystemExp turret;
    private final HoodSubsystemExp hood;
    private final FlywheelSubsystemExp flywheel;
    private final FeederSubsystemExp feeder;
    private final Supplier<Pose2d> robotPoseProvider;  
    private final Supplier<ChassisSpeeds> robotVelocityProvider;    

    public AutoAimAndShootCommandExp(
        TurretSubsystemExp turret, 
        HoodSubsystemExp hood, 
        FlywheelSubsystemExp flywheel, 
        FeederSubsystemExp feeder,
        Supplier<Pose2d> robotPoseProvider,
        Supplier<ChassisSpeeds> robotVelocityProvider) {

        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;
        this.feeder = feeder;
        this.robotPoseProvider = robotPoseProvider;    
        this.robotVelocityProvider = robotVelocityProvider;        
        
        // Command requires all these subsystems
        addRequirements(turret, hood, flywheel, feeder);
    }

    @Override
    public void execute() {

        Pose2d robotPose = robotPoseProvider.get();
        ChassisSpeeds robotVelocity = robotVelocityProvider.get();

        if (DriverStation.getAlliance().isPresent()) {
            Translation2d hubCoordinates = DriverStation.getAlliance().get() == Alliance.Blue ? 
            FieldElements.BLUE_HUB : FieldElements.RED_HUB;
        
            Translation2d targetTranslation = this.turret.getTurretTranslationToTarget(
                robotPose,
                hubCoordinates, 
                robotVelocity);
        
            turret.trackTarget(
                robotPose, 
                hubCoordinates, 
                robotVelocity); //Turret handles horizontal angle mapping

            double dist = targetTranslation.getNorm();
            
            hood.setAngleForDistance(dist);   // Hood handles vertical angle mapping
            flywheel.setRPMForDistance(dist); // Flywheel handles RPM mapping

            if (turret.isReadyToShoot(targetTranslation.getAngle().getDegrees(), robotVelocity) && 
                hood.isReadyToShoot() && flywheel.isReadyToShoot()) {
                feeder.run(1.0); // Pass 1.0 (Full Speed)
            }
            else {
                feeder.stop();
            }

        }




    }
}