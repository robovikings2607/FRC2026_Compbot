// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Amps;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.RobotLogger;

public abstract class MotorSubsystemBase extends SubsystemBase {
  protected final TalonFX motor;
  protected double targetMetric;

  protected final VoltageOut voltageRequest = new VoltageOut(0.0);
  protected final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  protected final PositionVoltage positionRequest = new PositionVoltage(0.0);  
  protected final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);

  
  public MotorSubsystemBase(int canId, TalonFXConfiguration config) {
    motor = new TalonFX(canId);
    motor.getConfigurator().apply(config);
   }

  /*
   * Tells the motor to run with a specific voltage
   * The Advantage: If you request 6 Volts, the Talon will push exactly 6 Volts whether the 
   * battery is fresh or dying. This guarantees consistent behavior across the entire match.
   *
   * When to use it: Highly recommended over DutyCycleOut for almost everything. 
   * It is essential for SysId characterization and open-loop drivetrains.
   * 
   * Volts.of(volts) to convert double to Voltage
   *
   */
  public void runWithVoltage(Voltage volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  /*
   * What it does: You command a target speed in Rotations Per Second (RPS). 
   * If a heavy game piece enters the mechanism and bogs the motor down, the PID loop will instantly detect the speed drop and aggressively spike the voltage to get back to the target RPS.
   *
   * When to use it: Anything that needs to maintain a constant speed under varying loads. 
   * Flywheels and Swerve Drive wheels.
   *    
   * RPM.of(velocity); to convert double to AngularVelocity
   * or
   * RevolutionsPerSecond.of(velocity); to convert double to AngularVelocity
   *
   */
  public void runAtSpeed(AngularVelocity velocity) {
    motor.setControl(velocityRequest.withVelocity(velocity));
  }

  /*
   * What it does: You command a target location in Rotations. The motor will drive to that exact sensor 
   * count and physically hold itself there, fighting anyone who tries to push it off target.
   *
   * When to use it: Mechanisms that move to specific setpoints but are relatively lightweight or 
   * have short travel distances. Adjustable hoods, swerve steering (azimuth) motors, and small turrets.
   *    
   * Degrees.of(degrees) to convert double to Angle
   *
   */
  public void runToLocation(Angle angle) {
    motor.setControl(positionRequest.withPosition(angle));
  }

  /*
   * What it does: This is a "Profiled" position request. If you use standard PositionVoltage to move 
   * a heavy elevator 10 rotations, the PID loop will violently slam full power instantly, which can 
   * snap chains and sheer gear teeth. MotionMagic calculates a smooth "Trapezoidal Profile"—it smoothly 
   * ramps up the acceleration, cruises at a max speed, and smoothly decelerates as it approaches the 
   * target.
   *
   * When to use it: Any heavy mechanism with physical limits. Elevators, heavy pivoting arms, 
   * and large climbing mechanisms.
   *    *    
   * Degrees.of(degrees) to convert double to Angle
   *
   */
  public void runToLocationMagicMotion(Angle angle) {
    motor.setControl(motionMagicRequest.withPosition(angle));
  }

  @Override
  public void periodic() {

    RobotLogger.logDouble(getNTKey() + "TargetVelocity", targetMetric);
    RobotLogger.logDouble(getNTKey() + "ActualVelocity", motor.getVelocity().getValueAsDouble());
    RobotLogger.logDouble(getNTKey() + "MotorVoltage", motor.getMotorVoltage().getValueAsDouble());    

  }

  public abstract String getNTSubsystemKey();
  
  public String getNTKey() {
    return getNTSubsystemKey() + "/";
  }

}
