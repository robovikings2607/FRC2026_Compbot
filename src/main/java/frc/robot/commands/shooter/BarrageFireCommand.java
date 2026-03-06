package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystemExp;
import frc.robot.subsystems.FlywheelSubsystemExp;
import frc.robot.subsystems.SpindexerSubsystemExp;

public class BarrageFireCommand extends Command {
    private final FlywheelSubsystemExp shooter;
    private final FeederSubsystemExp feeder;
    private final SpindexerSubsystemExp spindexer;
    
    private final double targetRPS;
    private final Timer unjamTimer = new Timer();

    // Define the three possible states of our sequence
    private enum FireState {
        SPOOLING,
        SHOOTING,
        UNJAMMING
    }
    
    private FireState currentState;

    public BarrageFireCommand(
        FlywheelSubsystemExp shooter, 
        FeederSubsystemExp feeder, 
        SpindexerSubsystemExp spindexer, 
        double targetRPS) {

        this.shooter = shooter;
        this.feeder = feeder;
        this.spindexer = spindexer;
        this.targetRPS = targetRPS;
        
        // Require all three subsystems so nothing else can interfere
        addRequirements(shooter, feeder, spindexer);
    }

    @Override
    public void initialize() {
        // Start the sequence by spooling the flywheel
        currentState = FireState.SPOOLING;
        shooter.setRPS(targetRPS);
        
        // Ensure feeder and spindexer start stopped
        feeder.stop();
        spindexer.stop();
    }

    @Override
    public void execute() {
        // The State Machine Logic
        switch (currentState) {
            
            case SPOOLING:
                // Wait for the flywheel to reach steady-state RPM
                if (shooter.isReadyToShoot()) {
                    currentState = FireState.SHOOTING;
                }
                break;

            case SHOOTING:
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

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        feeder.stop();
        spindexer.stop();
    }
}