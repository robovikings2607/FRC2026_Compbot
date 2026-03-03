package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldLocations;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystemExp;
import frc.robot.subsystems.FlywheelSubsystemExp;
import frc.robot.subsystems.HoodSubsystemExp;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystemExp;
import frc.robot.utilities.AimingCalculator;
import frc.robot.utilities.AimingSolution;
import frc.robot.utilities.ShooterState;

public class AutoAimAndShootCommandExp extends Command {
    private final TurretSubsystemExp turret;
    private final HoodSubsystemExp hood;
    private final FlywheelSubsystemExp flywheel;
    private final Supplier<Pose2d> robotPoseProvider;  
    private final Supplier<ChassisSpeeds> robotVelocityProvider;    
    private final AimingCalculator aimingMap = new AimingCalculator();

    public AutoAimAndShootCommandExp(
        TurretSubsystemExp turret, 
        HoodSubsystemExp hood, 
        FlywheelSubsystemExp flywheel, 
        Supplier<Pose2d> robotPoseProvider,
        Supplier<ChassisSpeeds> robotVelocityProvider) {

        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;
        this.robotPoseProvider = robotPoseProvider;    
        this.robotVelocityProvider = robotVelocityProvider;        
        
        // Command requires all these subsystems
        addRequirements(turret, hood, flywheel);
    }

    @Override
    public void execute() {

        Pose2d robotPose = robotPoseProvider.get();
        ChassisSpeeds robotVelocity = robotVelocityProvider.get();

        if (DriverStation.getAlliance().isPresent()) {
            Translation2d hubCoordinates = DriverStation.getAlliance().get() == Alliance.Blue ? 
            FieldLocations.BLUE_HUB : FieldLocations.RED_HUB;
        
            Translation2d turretCoordinates = turret.getTurretFieldPosition(robotPose); 

            AimingSolution solution = aimingMap.calculateMovingAimingSolution(
                turretCoordinates, 
                hubCoordinates, 
                robotVelocity
            );

            turret.trackTarget(solution.virtualTarget(), robotPose);
            hood.setAngle(solution.shooterState().hoodAngle);
            flywheel.setRPS(solution.shooterState().rps);
        }
    }
}