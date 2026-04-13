package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldLocations;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.MetricUnitsNameConstants;
import frc.robot.utilities.RobotLogger;
import frc.robot.utilities.ShooterUtils;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.Supplier;

public class FlywheelSubsystemTemplate extends MotorSubsystemBase{

    private static final double GEAR_RATIO = 1.0;
    private double targetRPM;   
    private InterpolatingDoubleTreeMap flywheelInterp = new InterpolatingDoubleTreeMap();
    private final Supplier<Pose2d> robotPoseSupplier;

    public FlywheelSubsystemTemplate(Supplier<Pose2d> robotPoseSupplier) {
        super(FlywheelConstants.FLYWHEEL_ID, getMotorConfiguration(), GEAR_RATIO);

        this.robotPoseSupplier = robotPoseSupplier;
    }

    protected String getNTSubsystemKey() {
        return "Flywheel";
    }

    protected String getMetricUnitName() {
        return MetricUnitsNameConstants.RPS;
    }
  

    protected double getTargetMetric() {
        return targetRPM;
    }
    
  @Override
  public void periodic() {
    Pose2d robotPose = robotPoseSupplier.get();

    Translation2d goalPose = FieldLocations.BLUE_HUB;

    double distance = robotPose.getTranslation().getDistance(goalPose);
    targetRPM = getGoal(distance);

    logDouble("Distance", distance);
    logCoreMotorMetrics();
  }

    protected double getActualMetric() {
        return getActualVelocityRPS();
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
    
    public void createInterpMap(){
        //key = distance from goal
        //value = speed of flywheel in rps 
        flywheelInterp.put(0.0, -40.0);
        flywheelInterp.put(1.5, -40.0);
        flywheelInterp.put(2.0, -42.5);
        flywheelInterp.put(2.5, -45.0);
        flywheelInterp.put(3.0, -47.5);
        flywheelInterp.put(3.5, -50.0);
        flywheelInterp.put(4.0, -52.5);
        flywheelInterp.put(4.5, -54.0);
        flywheelInterp.put(5.0, -56.5);
        flywheelInterp.put(5.5, -61.0);
        flywheelInterp.put(5.8, -61.0);
        //shootingInterp.put(6.0, 0.0);
    }

    public double getGoal(double distance){
        return flywheelInterp.get(distance);
    }

}
