package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystemExp;
import frc.robot.subsystems.HoodSubsystemExp;
import frc.robot.subsystems.TurretSubsystem;

public class AutoAimAndShootCommandExp extends Command {
    private final TurretSubsystem turret;
    private final HoodSubsystemExp hood;
    private final FlywheelSubsystemExp flywheel;
    private final FeederSubsystem feeder;
    private final DoubleSupplier distanceSupplier;

    public AutoAimAndShootCommandExp(TurretSubsystem turret, HoodSubsystemExp hood, 
                          FlywheelSubsystemExp flywheel, FeederSubsystem feeder,
                          DoubleSupplier distanceSupplier) {
        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;
        this.feeder = feeder;
        this.distanceSupplier = distanceSupplier;
        
        // Command requires all these subsystems
        addRequirements(turret, hood, flywheel, feeder);
    }

    @Override
    public void execute() {
        // 1. Get the single source of truth (Distance)
        double dist = distanceSupplier.getAsDouble();

        // 2. Broadcast distance to subsystems
        // turret.trackTarget();       // Turret handles aiming
        // hood.setTargetDistance(dist);   // Hood handles Angle mapping
        // flywheel.setTargetDistance(dist); // Flywheel handles RPM mapping

        // // 3. Check if everyone is ready
        // if (turret.isAligned() && hood.isAtTarget() && flywheel.isAtTarget()) {
        //     feeder.run();
        // } else {
        //     feeder.stop();
        // }
    }
}