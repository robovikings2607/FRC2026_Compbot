package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldLocations;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.MetricUnitsNameConstants;
import frc.robot.utilities.RobotLogger;
import frc.robot.utilities.ShooterUtils;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

public class FlywheelSubsystemTemplate extends MotorSubsystemBase{

    private static final double GEAR_RATIO = 1.0;
    private double targetRPM;   
    private InterpolatingDoubleTreeMap flywheelInterp = new InterpolatingDoubleTreeMap();
    private final Supplier<Pose2d> robotPoseSupplier;

    private final double kMomentOfInertia = 0.006; // Estimated mass/inertia of the feeder wheel (kg*m^2)
    private final TalonFXSimState motorSimState = motor.getSimState();
    private final FlywheelSim flywheelSim;


    public FlywheelSubsystemTemplate(Supplier<Pose2d> robotPoseSupplier) {
        super("Flywheel", FlywheelConstants.FLYWHEEL_ID, getMotorConfiguration(), GEAR_RATIO);

        this.robotPoseSupplier = robotPoseSupplier;

        var flywheelPlant = LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1), // Motor
                kMomentOfInertia,        // J (kg * m^2)
                GEAR_RATIO               // Gearing
        );

        // 2. Initialize the simulator with the plant
        feederPhysicsSim = new FlywheelSim(
            flywheelPlant,           // The physics model we just created
            DCMotor.getKrakenX60(1), // The motor type (used by sim to calculate current draw)
            GEAR_RATIO               // The gearing
        );    
        
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

    @Override
    protected void updateMechanismSimulation(double appliedVolts, double dtSeconds) {

        flywheelSim.setInputVoltage(appliedVolts);
        flywheelSim.update(dtSeconds);

        AngularVelocity mechanismVelocity = flywheelSim.getAngularVelocity();

        simState.setRotorVelocity(mechanismVelocity.in(RotationsPerSecond));
    }

}
