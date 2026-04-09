// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.PortUnreachableException;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utilities.ISysIdTunable;
import frc.robot.utilities.SysIdBuilder;
import frc.robot.Constants.FeederConstants;

import static edu.wpi.first.units.Units.*;

public class FeederSubsystem extends SubsystemBase implements ISysIdTunable {
  /** Creates a new FeederSubsystem. */
  //private int reverse;
  private TalonFX motor;
  private VelocityVoltage control = new VelocityVoltage(0.0);
  private double speed;

  //Simulation code
  private final double kGearRatio = 1.0;
  private final double kMomentOfInertia = 0.001; // Estimated mass/inertia of the feeder wheel (kg*m^2)
  private final TalonFXSimState motorSimState = motor.getSimState();
  private final FlywheelSim feederPhysicsSim;


  public FeederSubsystem(RobotContainer robot) {
    motor = new TalonFX(FeederConstants.FEEDER_ID);
    configureMotor();

    var flywheelPlant = LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX44(1), // Motor
            kMomentOfInertia,        // J (kg * m^2)
            kGearRatio               // Gearing
    );

    // 2. Initialize the simulator with the plant
    feederPhysicsSim = new FlywheelSim(
        flywheelPlant,           // The physics model we just created
        DCMotor.getKrakenX44(1), // The motor type (used by sim to calculate current draw)
        kGearRatio               // The gearing
    );    

  }

  private final SysIdRoutine sysIdRoutine = SysIdBuilder.buildTalonFXRoutine(
        motor, this, "feeder", 4.0
  );    

  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    speed = SmartDashboard.getNumber("Feeder/Speed", 0.0);

  }

  public void configureMotor(){
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(85))
            .withStatorCurrentLimitEnable(true)
    );

    var slot0Configs = configs.Slot0;
          // slot0Configs.kS = 0.0; // Voltage output to overcome static friction
          slot0Configs.kV = 0.1075; // A velocity target of 1 rps requires this voltage output.
          // slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires this voltage output
          slot0Configs.kP = 0.5; // A position error of 2.5 rotations requires this voltage output
          slot0Configs.kI = 0; // no output for integrated error
          slot0Configs.kD = 0.000; // A velocity error of 1 rps requires this voltage output

    configs.withSlot0(slot0Configs);

    motor.getConfigurator().apply(configs);
  }

  public void runMotor() {
    //feederMotor.setVoltage(FeederConstants.FEEDER_SPEED);
    motor.setControl(control.withVelocity(75));
  }

  public void stopMotor() {
    motor.setControl(new CoastOut());
  }

  public void reverseMotor() {
    motor.setVoltage(-FeederConstants.FEEDER_SPEED);;
  }

  public double getSimulatedCurrentDraw() {
    return feederPhysicsSim.getCurrentDrawAmps();
  }
  
  @Override
  public void simulationPeriodic() {
      // 1. Give the simulated motor a virtual battery voltage
      motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

      // 2. Find out what voltage your robot code is telling the motor to apply
      double appliedVoltage = motorSimState.getMotorVoltage();

      // 3. Feed that applied voltage into the WPILib physics model
      feederPhysicsSim.setInputVoltage(appliedVoltage);

      // 4. Advance the physics model by the standard 20ms robot loop
      feederPhysicsSim.update(0.020);

      // 5. Extract the resulting velocity from the physics model
      // NOTE: WPILib physics returns radians per second at the *mechanism*
      double mechanismVelocityRadPerSec = feederPhysicsSim.getAngularVelocityRadPerSec();

      // 6. Convert to the units CTRE expects: Rotations Per Second (RPS) at the *rotor*
      double mechanismVelocityRps = mechanismVelocityRadPerSec / (2 * Math.PI);
      double rotorVelocityRps = mechanismVelocityRps * kGearRatio;

      // 7. Update the virtual encoder
      motorSimState.setRotorVelocity(rotorVelocityRps);
  }

}