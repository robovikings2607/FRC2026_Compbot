package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import com.ctre.phoenix6.hardware.TalonFX;


public abstract class ShooterSubsystemExp extends SubsystemBase {
    protected TalonFX motor;
    protected RobotContainer robot;
    protected double currentTargetRotations = 0.0;

    public ShooterSubsystemExp(RobotContainer robot, int canId) {
      this.robot = robot;
      this.motor = new TalonFX(canId);
      configureMotor();
    }


    public void setCurrentRotations(double currentTargetRotations) {
        this.currentTargetRotations = currentTargetRotations;
    }

    protected boolean isAtTarget() {
      double currentPos = motor.getPosition().getValueAsDouble();

      double error = Math.abs(currentTargetRotations - currentPos);

      return error < getTargetTolerance();
    }

    public abstract void configureMotor();

    public boolean isReadyToShoot() {
      return isAtTarget();
    }

    public abstract double getTargetTolerance();  
    
}