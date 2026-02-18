package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystemExp;

public class TuneFlywheelCommand extends Command {
    private final FlywheelSubsystemExp flywheel;
    
    // Define the exact name of the text box that will appear on your dashboard
    private final String tuningKey = "Tuning/TargetRPS";

    public TuneFlywheelCommand(FlywheelSubsystemExp flywheel) {
        this.flywheel = flywheel;

        addRequirements(flywheel); 
        
        SmartDashboard.setDefaultNumber(tuningKey, 0.0);
    }

    @Override
    public void execute() {
        // 1. Read the current number from the dashboard
        double targetRPS = SmartDashboard.getNumber(tuningKey, 0.0);
        
        // 2. Send that target to your subsystem's existing Velocity/Voltage method
        flywheel.setRPS(targetRPS);
    }

    @Override
    public void end(boolean interrupted) {
        // Always stop the dangerous fast-spinning mechanism when the command ends
        flywheel.stop(); 
    }
}