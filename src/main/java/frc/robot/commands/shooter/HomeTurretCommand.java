package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystemExp;

public class HomeTurretCommand extends Command {
    private final TurretSubsystemExp turret;
    private final Timer m_timer = new Timer();
    
    // Configurable constants for your specific mechanism
    private static final double HOMING_VOLTAGE = -1.5; // Negative to drive towards the left hard stop
    private static final double STALL_CURRENT_AMPS = 20.0; // The spike threshold (tune this!)
    private static final double LEFT_HARD_STOP_DEGREES = -130.0; // The known physical limit

    public HomeTurretCommand(TurretSubsystemExp turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        // Start the timer to handle the inrush current delay
        m_timer.restart();
    }

    @Override
    public void execute() {
        // Bypass the PID and apply a slow, gentle raw voltage
        turret.applyRawVoltage(HOMING_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
        // 1. Ignore the first 0.25 seconds to let the motor start spinning safely
        if (m_timer.get() < 0.25) {
            return false;
        }

        // 2. Check the Phoenix 6 Stator Current
        double currentAmps = turret.getTurretStatorCurrent();
        
        // 3. If the current spikes above our threshold, we hit the wall!
        return currentAmps >= STALL_CURRENT_AMPS;
    }

    @Override
    public void end(boolean interrupted) {
        // 1. Instantly kill the power so we don't cook the motor
        turret.applyRawVoltage(0.0);

        // 2. If the command finished naturally (we found the wall), reset the encoder!
        if (!interrupted) {
            turret.setEncoderToAngle(LEFT_HARD_STOP_DEGREES);
            System.out.println("Turret Successfully Homed to " + LEFT_HARD_STOP_DEGREES);
        }
    }
}