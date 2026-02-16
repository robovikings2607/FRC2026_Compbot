package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.FlywheelConstants;


public class FlywheelSubsystemExp extends ShooterSubsystemExp {
    private static final double GEAR_RATIO = 10.0; // 10 motor turns = 1 turret turn  
    private final MotionMagicVoltage magicMotionRequest = new MotionMagicVoltage(0);
    private final InterpolatingDoubleTreeMap distanceToRPMMap = new InterpolatingDoubleTreeMap();
    private final double TARGET_ERR_TOLERANCE = 0.01;  



    public FlywheelSubsystemExp(RobotContainer robot) {
      super(robot, FlywheelConstants.FLYWHEEL_ID);

      initializeDistanceToAngleMap();    
      configureMotor();
    }

    private void initializeDistanceToAngleMap() {
      distanceToRPMMap.put(1.0, 15.0);
      distanceToRPMMap.put(5.0, 55.0);
    }

    public void setRPMForDistance(double distanceMeters) {
        double targetRPM = distanceToRPMMap.get(distanceMeters);
        setRPM(targetRPM);
    }

    public void setRPM(double targetRPM) {
        double finalMotorSetpointRotations = targetRPM / GEAR_RATIO;
        
        motor.setControl(magicMotionRequest.withPosition(finalMotorSetpointRotations));
        SmartDashboard.putNumber("Hood/newSetPointRotations", finalMotorSetpointRotations);

        this.currentTargetRotations = finalMotorSetpointRotations;
    }

    @Override
    public void configureMotor(){

    TalonFXConfiguration configs = new TalonFXConfiguration();

    //If you set the SensorToMechanismRatio then the Pheonix does the math to convert
    //to motor rotations but you must then set everything inclduing
    //the pid values below in terms of mechanism rotations
    //configsFeedback.SensorToMechanismRatio = GEAR_RATIO;


    var slot0Configs = configs.Slot0;
        slot0Configs.kS = 0.25; // Voltage output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps requires this voltage output.
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires this voltage output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations requires this voltage output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.11; // A velocity error of 1 rps requires this voltage output

    var motionMagicConfigs = configs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 160; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 320; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

/*     //enable software limits
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    //limits (in rotations)
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = rotationsPerDegree * 120;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -rotationsPerDegree * 240; */
  
    motor.getConfigurator().apply(configs);
    motor.setNeutralMode(NeutralModeValue.Brake);

    motor.setPosition(0); //
  }

  @Override
  public double getTargetTolerance() {
    return TARGET_ERR_TOLERANCE;
  }
}