// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utilities.RobotLogger;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        // Wakes up the Logger class and triggers its static setup block
        // this will also call DataLogManager.start to initiate .wpilog logging 
        RobotLogger.init(); 

        // Force the Command Scheduler to broadcast its status to NetworkTables
        SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());        
    }


    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        
        /*
        if(RobotController.getUserButton()){
            m_robotContainer.hood.getMotor().setPosition(0.0);
            m_robotContainer.turret.getMotor().setPosition(0.0);
            m_robotContainer.intake.getPivotMotor().setPosition(0.0);
        } 
        */
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

// Inside Robot.java
@Override
public void simulationPeriodic() {
    // 1. Gather the current draw from all your simulated mechanisms
    double totalCurrentAmps = 0;
    totalCurrentAmps += m_robotContainer.feeder.getSimulatedCurrentDraw();
    // totalCurrentAmps += m_swerveDrive.getSimulatedCurrentDraw();
    // totalCurrentAmps += m_climber.getSimulatedCurrentDraw();

    // 2. Calculate what the battery voltage should drop to based on that load
    // WPILib uses a standard internal resistance model for an FRC battery (0.018 ohms)
    //double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentAmps);

    // 3. Update the global HAL battery voltage
    // Now, every call to RobotController.getBatteryVoltage() will return this lower number!
    //RoboRioSim.setVInVoltage(loadedVoltage);
}}
