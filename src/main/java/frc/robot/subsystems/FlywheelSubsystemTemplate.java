package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.MetricUnitsNameConstants;

import static edu.wpi.first.units.Units.Amps;

public class FlywheelSubsystemTemplate extends MotorSubsystemBase{

    private static final double GEAR_RATIO = 1.0;

    public FlywheelSubsystemTemplate() {
        super(FlywheelConstants.FLYWHEEL_ID, getMotorConfiguration(), GEAR_RATIO);
    }

    protected String getNTSubsystemKey() {
        return "Flywheel";
    }

    protected String getMetricUnitName() {
        return MetricUnitsNameConstants.RPM;
    }
  

    protected double getTargetMetric() {
        return 0.0;
    }
    
    protected double getActualMetric() {
        return getActualVelocityRpm();
    }


    private static TalonFXConfiguration getMotorConfiguration() { 
        TalonFXConfiguration config = new TalonFXConfiguration();

        var slot0Configs = config.Slot0;
            // slot0Configs.kS = 0.0; // Voltage output to overcome static friction
            slot0Configs.kV = 0.12; // A velocity target of 1 rps requires this voltage output.
            // slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires this voltage output
            slot0Configs.kP = 0.6; // A position error of 2.5 rotations requires this voltage output
            slot0Configs.kI = 0; // no output for integrated error
            slot0Configs.kD = 0.000; // A velocity error of 1 rps requires this voltage output

        config.withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
            );

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.SensorToMechanismRatio = 1.0;


        return config;
    }
    
}
