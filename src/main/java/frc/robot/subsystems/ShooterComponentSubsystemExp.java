package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public abstract class ShooterComponentSubsystemExp extends SubsystemBase {
    protected final TalonFX motor;
    protected final RobotContainer robot;
    protected final MotionMagicVoltage magicMotionRequest = new MotionMagicVoltage(0);    
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

    public void setCurrentRotations(double currentTargetRotations) {
        this.currentTargetRotations = currentTargetRotations;
    }

    protected boolean isAtTarget() {
      double currentPos = motor.getPosition().getValueAsDouble();

      double error = Math.abs(currentTargetRotations - currentPos);

      return error < getTargetTolerance();
    }

    protected abstract void configureMotor();

    public boolean isReadyToShoot() {
      return isAtTarget();
    }

    public abstract double getTargetTolerance();  
    
}