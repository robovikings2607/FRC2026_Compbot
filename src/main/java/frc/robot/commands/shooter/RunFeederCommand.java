package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class RunFeederCommand extends Command {
    
    private final FeederSubsystem feeder;
    private final double targetRps;

    // The Constructor: This runs ONCE when you bind the button
    public RunFeederCommand(FeederSubsystem feeder, double targetRps) {
        this.feeder = feeder;
        this.targetRps = targetRps;
        
        // CRITICAL: You must declare the subsystem requirement.
        // This stops two commands from trying to use the feeder at the same time.
        addRequirements(feeder); 
    }

    // Runs ONCE when the command is scheduled (when you press the button)
    @Override
    public void initialize() {
        // You could put a print statement here for debugging
        System.out.println("FEEDER COMMAND INITIATED!");    
        // Publish the state as a string to the exact same folder
        SmartDashboard.putString("Feeder/CommandState", "INITIATED");            
    }

    // Runs REPEATEDLY every 20ms while the button is held
    @Override
    public void execute() {
        feeder.runMotor(targetRps);
    }

    // Runs ONCE when the command is interrupted (when you let go of the button)
    @Override
    public void end(boolean interrupted) {
        feeder.stopMotor();

        if (interrupted) {
            SmartDashboard.putString("Feeder/CommandState", "INTERRUPTED (Button Released)");
        } else {
            SmartDashboard.putString("Feeder/CommandState", "FINISHED");
        }        
    }

    // Tells the Command Scheduler when to stop.
    @Override
    public boolean isFinished() {
        // Returning false means it runs infinitely until interrupted by letting go of the button.
        return false; 
    }
}