package frc.robot.utilities;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysIdBuilder {

    /**
     * Builds a standardized SysIdRoutine for any TalonFX motor.
     * @param motor         The TalonFX motor to test.
     * @param subsystem     The subsystem this motor belongs to (for command requirements).
     * @param mechanismName The string name for the log file (e.g., "flywheel", "feeder").
     * @param stepVoltage   The dynamic test step voltage (e.g., 7.0 for heavy, 4.0 for light).
     * @return A fully configured SysIdRoutine.
     */
    public static SysIdRoutine buildTalonFXRoutine(
            TalonFX motor, 
            Subsystem subsystem, 
            String mechanismName, 
            double stepVoltage) {

        // Create the control request once for this routine
        VoltageOut sysIdControl = new VoltageOut(0);

        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(stepVoltage), // Pass in your custom safe voltage
                null, 
                null
            ), 
            new SysIdRoutine.Mechanism(
                // Tell SysId how to apply voltage
                (voltage) -> motor.setControl(sysIdControl.withOutput(voltage.in(Volts))),
                
                // Tell SysId how to log the TalonFX data
                (log) -> {
                    log.motor(mechanismName)
                       .voltage(Volts.of(motor.getMotorVoltage().getValueAsDouble()))
                       .angularPosition(Rotations.of(motor.getPosition().getValueAsDouble()))
                       .angularVelocity(RotationsPerSecond.of(motor.getVelocity().getValueAsDouble()));
                },
                
                // Pass the subsystem requirement
                subsystem 
            )
        );
    }
}