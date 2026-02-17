package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public abstract class ShooterComponentSubsystemExp extends SubsystemBase {
    protected final TalonFX motor;
    protected final RobotContainer robot;
    protected final MotionMagicVoltage magicMotionRequest = new MotionMagicVoltage(0).withSlot(0); 
    private final PositionVoltage  positionRequest = new PositionVoltage(0).withSlot(1);

    protected double currentTargetRotations = 0.0;

    public ShooterComponentSubsystemExp(RobotContainer robot, int canId) {
      this.robot = robot;
      this.motor = new TalonFX(canId);
      configureMotor();
    }

    protected void SetMotorPosition(double motorSetpointRotations, String smartDashBoardKey) {
        motor.setControl(magicMotionRequest.withPosition(motorSetpointRotations));
        SmartDashboard.putNumber(smartDashBoardKey, motorSetpointRotations);

        this.currentTargetRotations = motorSetpointRotations;
    }

    protected void SetAgressiveMotorPosition(double motorSetpointRotations, String smartDashBoardKey) {
        motor.setControl(magicMotionRequest.withPosition(motorSetpointRotations));
        motor.setControl(positionRequest.withPosition(motorSetpointRotations));

        SmartDashboard.putNumber(smartDashBoardKey, motorSetpointRotations);

        this.currentTargetRotations = motorSetpointRotations;
    }

    public void setCurrentRotations(double currentTargetRotations) {
        this.currentTargetRotations = currentTargetRotations;
    }

    protected boolean isAtTarget() {
      double currentPos = motor.getPosition().getValueAsDouble();

      double error = Math.abs(currentTargetRotations - currentPos);

      return error < getTargetTolerance();
    }

    public boolean isReadyToShoot() {
      return isAtTarget();
    }

    public boolean isSlot1Configured() {
      var readConfigs = new Slot1Configs();

      var status = motor.getConfigurator().refresh(readConfigs);

      if (!status.isOK()) {
          return false;
      }

      boolean isKPZero = Math.abs(readConfigs.kP) < 0.001;
      boolean isKVZero = Math.abs(readConfigs.kV) < 0.001;

      if (isKPZero && isKVZero) {
          return false;
      }

      return true;
    }
    protected abstract void configureMotor();

    public abstract double getTargetTolerance();  
    
}