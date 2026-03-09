package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldLocations;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystemExp;
import frc.robot.subsystems.FlywheelSubsystemExp;
import frc.robot.subsystems.HoodSubsystemExp;
import frc.robot.subsystems.SpindexerSubsystemExp;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystemExp;
import frc.robot.utilities.AimingCalculator;
import frc.robot.utilities.AimingSolution;
import frc.robot.utilities.ShooterState;

public class AutoAimAndShootCommandExp extends Command {
    private final TurretSubsystemExp turret;
    private final HoodSubsystemExp hood;
    private final FlywheelSubsystemExp flywheel;
    private final FeederSubsystemExp feeder;
    private final SpindexerSubsystemExp spindexer;
    private final Supplier<Pose2d> robotPoseProvider;  
    private final Supplier<ChassisSpeeds> robotVelocityProvider;    
    private final AimingCalculator aimingMap = new AimingCalculator();
    private final Timer unjamTimer = new Timer();    

    // Define the three possible states of our sequence
    private enum FireState {
        SPOOLING,
        SHOOTING,
        UNJAMMING
    }

    private FireState currentState;

    public AutoAimAndShootCommandExp(
        TurretSubsystemExp turret, 
        HoodSubsystemExp hood, 
        FlywheelSubsystemExp flywheel, 
        FeederSubsystemExp feeder,
        SpindexerSubsystemExp spindexer,
        Supplier<Pose2d> robotPoseProvider,
        Supplier<ChassisSpeeds> robotVelocityProvider) {

        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;
        this.feeder = feeder;        
        this.spindexer = spindexer;                
        this.robotPoseProvider = robotPoseProvider;    
        this.robotVelocityProvider = robotVelocityProvider;        
        
        addRequirements(turret, hood, flywheel, feeder, spindexer);
    }

    @Override
    public void execute() {
        
        currentState = FireState.SPOOLING;

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

        switch (currentState) {
            
            case SPOOLING:
                feeder.stop();
                spindexer.stop();                
                
                if (turret.isReadyToShoot() && hood.isReadyToShoot() &&  flywheel.isReadyToShoot()) {
                    currentState = FireState.SHOOTING;
                }
                break;

            case SHOOTING:
                if (!(turret.isReadyToShoot() && hood.isReadyToShoot() &&  flywheel.isReadyToShoot())) {
                    currentState = FireState.SPOOLING;
                    break;
                }

                // Run the feeder fast to pull balls away
                feeder.runForwardAtTunedSpeed(); 
                
                // Run the spindexer at a tuned, constant RPS to feed the balls
                spindexer.runForwardAtTunedSpeed(); 
     
                // Watch for a current spike indicating a pinch at the exit hole
                if (spindexer.isJammed()) {
                    System.out.println("Jam detected! Initiating hiccup.");
                    currentState = FireState.UNJAMMING;
                    unjamTimer.restart(); // Start counting the unjam duration
                }
                break;

            case UNJAMMING:
                // Keep running the feeder forward to clear the throat
                feeder.runForwardAtTunedSpeed(); //run at 100%
                
                // Briefly reverse the spindexer to break the pinch
                spindexer.runGentleReverse();
                
                // After 0.25 seconds of reversing, go back to shooting
                if (unjamTimer.hasElapsed(0.25)) {
                    System.out.println("Hiccup complete. Resuming fire.");
                    unjamTimer.stop();
                    currentState = FireState.SHOOTING;
                }
                break;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        spindexer.stop();
    }
}