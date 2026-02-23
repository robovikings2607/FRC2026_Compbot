// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HoodSubsystemExp;
import frc.robot.utilities.GeometryUtil;

public class ZeroHoodCommand extends Command {

  RobotContainer robot;
  HoodSubsystemExp hood;

    // --- CALIBRATION CONSTANTS ---
    // 1. A small, safe voltage to push the hood down (Negative = Down)
    private static final double HOMING_VOLTAGE = -2.0; 
    
    // 2. The Current Spike Threshold (Amps)
    // A free-spinning hood uses ~2-5 Amps. A stalled hood spikes to 20-40+ Amps.
    private static final double STALL_CURRENT_AMPS = 20.0;
    
    // 3. The Physical Angle (Degrees)
    // Measure this with a digital level or phone app when the hood is resting on the bottom stop!
    private static final double KNOWN_BOTTOM_ANGLE_DEGREES = 28.5; 

    private final VoltageOut voltageRequest = new VoltageOut(HOMING_VOLTAGE);


  public ZeroHoodCommand(RobotContainer robot) {
    this.robot = robot;
    hood = robot.hoodExp;

    addRequirements(hood);
  }

  @Override
    public void initialize() {
        // Start gently driving the hood downward
        hood.getMotor().setControl(voltageRequest.withOutput(HOMING_VOLTAGE));
    }

    @Override
    public void execute() {
        // The motor is continuously pushing in the background; nothing needed here.
    }

    @Override
    public boolean isFinished() {
        // Stator current is the most accurate way to detect a mechanical stall.
        double currentAmps = hood.getMotor().getStatorCurrent().getValueAsDouble();
        
        // If the current spikes above our threshold, we have hit the hard stop!
        return currentAmps >= STALL_CURRENT_AMPS;
    }

    @Override
    public void end(boolean interrupted) {

        hood.getMotor().setControl(new VoltageOut(0));

        if (!interrupted) {
            double rotations = GeometryUtil.getMotorRotationsAsDegrees(KNOWN_BOTTOM_ANGLE_DEGREES, HoodSubsystemExp.GEAR_RATIO);
            hood.getMotor().setPosition(rotations);
            
            System.out.println("Hood Zeroed Successfully at " + KNOWN_BOTTOM_ANGLE_DEGREES + " degrees!");
        }
    }
}
