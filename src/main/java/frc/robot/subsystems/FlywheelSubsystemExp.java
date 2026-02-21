package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.FlywheelConstants;
import frc.robot.utilities.ShooterUtils;


public class FlywheelSubsystemExp extends SubsystemBase {
    private static final double GEAR_RATIO = 1.0;
    private final double TARGET_ERR_TOLERANCE_RPS = 1.5;  
    protected final TalonFX motor;
    protected final RobotContainer robot;
    private VelocityDutyCycle velocityControl = new VelocityDutyCycle(0);   
    // Create a Debouncer: RPS must be true for 0.1 seconds continuously
    private final Debouncer readyDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    private double targetRPS = 0.0;    


    public FlywheelSubsystemExp(RobotContainer robot) {
      this.robot = robot;
      this.motor = new TalonFX(FlywheelConstants.FLYWHEEL_ID);

      configureMotor();
    }

    public void setRPS(double targetRPS) {
        double adjustedTargetRPS = targetRPS / GEAR_RATIO;

        SetMotorPosition(adjustedTargetRPS, "Flywheel/newTargetRPS");
        targetRPS = ShooterUtils.getRPS(adjustedTargetRPS);
    }

    protected void SetMotorPosition(double targetRPS, String smartDashBoardKey) {
        motor.setControl(velocityControl.withVelocity(targetRPS));
        SmartDashboard.putNumber(smartDashBoardKey, targetRPS);
    }

    protected boolean isAtTarget() {
        double currentRPS = motor.getVelocity().getValueAsDouble();
        
        boolean isWithinTolerance = Math.abs(currentRPS - targetRPS) < getTargetTolerance();
        
        return readyDebouncer.calculate(isWithinTolerance);
    }

    public boolean isReadyToShoot() {
      return isAtTarget();
    }

    public void stop() {
      motor.stopMotor();
    }

    protected void configureMotor(){

      TalonFXConfiguration configs = new TalonFXConfiguration();

      // ---------------------------------------------------------
      // 1. CURRENT LIMITS (Protecting the battery and the motor)
      // ---------------------------------------------------------
      CurrentLimitsConfigs currentLimits = configs.CurrentLimits;

      // Supply Limit: Protects your 40A breaker and prevents brownouts.
      currentLimits.SupplyCurrentLimitEnable = true;
      currentLimits.SupplyCurrentLimit = 60.0;      // Continuous limit (Amps)
      currentLimits.SupplyCurrentLowerLimit = 40.0;  // Drop down to a safe 40A continuous...
      currentLimits.SupplyCurrentLowerTime = 0.1;    // ...if the spike lasts longer than 0.1 seconds.

      // Stator Limit: Protects the physical motor coils from melting if jammed.
      currentLimits.StatorCurrentLimitEnable = true;
      currentLimits.StatorCurrentLimit = 80.0;      // Amps (Torque limit)

      // ---------------------------------------------------------
      // 2. DIRECTIONAL LIMITS (Preventing reverse braking)
      // ---------------------------------------------------------
      MotorOutputConfigs outputConfigs = configs.MotorOutput;

      // Force the motor to only ever output positive power or coast.
      // This stops aggressive PID tuning from spitting a piece backward.
      outputConfigs.PeakReverseDutyCycle = 0.0;      

      var slot0Configs = configs.Slot0;
            slot0Configs.kS = 0.2; // Voltage output to overcome static friction
            slot0Configs.kV = 0.096; // A velocity target of 1 rps requires this voltage output.
            slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires this voltage output
            slot0Configs.kP = 0.05; // A position error of 2.5 rotations requires this voltage output
            slot0Configs.kI = 0; // no output for integrated error
            slot0Configs.kD = 0.01; // A velocity error of 1 rps requires this voltage output

      motor.getConfigurator().apply(configs);
      motor.setNeutralMode(NeutralModeValue.Coast);

      motor.setPosition(0); //
    }

  public double getTargetTolerance() {
    return TARGET_ERR_TOLERANCE_RPS;
  }
}