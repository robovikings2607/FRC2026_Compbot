package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystemExp;
import frc.robot.subsystems.FlywheelSubsystemExp;
import frc.robot.subsystems.HoodSubsystemExp;

public class ManualShootCommandExp extends Command {
    private final HoodSubsystemExp hood;
    private final FlywheelSubsystemExp flywheel;
    private final FeederSubsystemExp feeder;
    
    private final double targetRPM;
    private final double targetAngle;

    public ManualShootCommandExp(HoodSubsystemExp hood, FlywheelSubsystemExp flywheel, 
                              FeederSubsystemExp feeder, 
                              double targetRPM, double targetAngle) {
        this.hood = hood;
        this.flywheel = flywheel;
        this.feeder = feeder;
        this.targetRPM = targetRPM;
        this.targetAngle = targetAngle;
        
        addRequirements(hood, flywheel, feeder);
    }

    @Override
    public void execute() {
        // Direct control: Bypass the "Distance" logic
        hood.setAngle(targetAngle); 
        flywheel.setRPS(targetRPM);

        // Simple check: Are we there yet?
        if (hood.isReadyToShoot() && flywheel.isReadyToShoot()) {
            feeder.runForwardAtTunedSpeed(); 
        }
    }
}