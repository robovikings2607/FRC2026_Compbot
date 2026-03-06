// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SpindexerConstants;

public class SpindexerSubsystemExp extends SubsystemBase {
  private TalonFX motor;
  private final DutyCycleOut driveRequest = new DutyCycleOut(0);      
  // We use a VelocityVoltage request to force a constant RPM
  private final VelocityVoltage velocityReq = new VelocityVoltage(0).withSlot(0);
  
  // This is the "Goldilocks" speed you found during testing
  // e.g., 1.5 Rotations Per Second (90 RPM)
  private final double TUNED_DELIVERY_RPS = 1.5; 



  public SpindexerSubsystemExp(RobotContainer robot) {
    motor = new TalonFX(SpindexerConstants.SPINDEXER_ID);
  }

  @Override
  public void periodic() {

  }

  /**
   * Runs the spindexer at a specific percent speed.
   * @param percent -1.0 to 1.0
   */
  public void run(double percent) {
      motor.setControl(driveRequest.withOutput(percent));
  }

  public void stop() {
      motor.setControl(driveRequest.withOutput(0));
  }

  // Stator current measures the torque the motor is applying. 
  // When it spikes, you are jammed.
  public double getCurrent() {
      return motor.getStatorCurrent().getValueAsDouble();
  }

  public boolean isJammed() {
      // If the motor is pulling more than 40 Amps, it is physically stuck.
      // (You will need to tune this 40 Amp threshold by looking at your logs!)
      return getCurrent() > 40.0; 
  }

      public void runForwardAtTunedSpeed() {
        // Send the velocity request to the motor
        motor.setControl(velocityReq.withVelocity(TUNED_DELIVERY_RPS));
    }

    /**
     * Runs the spindexer in reverse using raw voltage to clear a jam.
     */
    public void runGentleReverse() {
        // We can just use raw voltage/percent output for unjamming
        // since we just need a quick, generic backward tug.
        motor.set(-0.30); 
    }

}
