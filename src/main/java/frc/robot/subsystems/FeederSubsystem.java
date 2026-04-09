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
import frc.robot.utilities.RobotLogger;
import frc.robot.utilities.SysIdBuilder;
import frc.robot.Constants.FeederConstants;

import static edu.wpi.first.units.Units.*;

public class FeederSubsystem extends SubsystemBase implements ISysIdTunable {
  private TalonFX motor = new TalonFX(FeederConstants.FEEDER_ID);
  private VelocityVoltage control = new VelocityVoltage(0.0);
  private double targetVelocityRps = 0.0;  

  //Simulation code
  private final double kGearRatio = 1.0;
  private final double kMomentOfInertia = 0.001; // Estimated mass/inertia of the feeder wheel (kg*m^2)
  private final TalonFXSimState motorSimState = motor.getSimState();
  private final FlywheelSim feederPhysicsSim;


  public FeederSubsystem(RobotContainer robot) {

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
      // 2. Log the Target Velocity
      RobotLogger.logDouble("Feeder/TargetVelocity", targetVelocityRps);

      // 3. Log the Actual Velocity (Using Phoenix 6 syntax)
      RobotLogger.logDouble("Feeder/ActualVelocity", motor.getVelocity().getValueAsDouble());

      // 4. Log the Control Effort (Voltage) for tuning
      RobotLogger.logDouble("Feeder/MotorVoltage", motor.getMotorVoltage().getValueAsDouble());
  }

  public void configureMotor(){
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(85))
            .withStatorCurrentLimitEnable(true)
    );

    var slot0Configs = configs.Slot0;
          // slot0Configs.kS = 0.0;
          slot0Configs.kV = 0.100; 
          // slot0Configs.kA = 0.0; 
          slot0Configs.kP = 0.0; 
          slot0Configs.kI = 0; 
          slot0Configs.kD = 0.000;

    configs.withSlot0(slot0Configs);

    motor.getConfigurator().apply(configs);
  }

  public void runMotor() {
    runMotor(75);
  }

  public void runMotor(double rps) {
    targetVelocityRps = rps;
    motor.setControl(control.withVelocity(rps));
  }
  public void stopMotor() {
    targetVelocityRps = 0.0;    
    motor.setControl(new CoastOut());
  }

  public void reverseMotor() {
    reverseMotor(FeederConstants.FEEDER_SPEED);;
  }

  public void reverseMotor(double rps) {
    targetVelocityRps = rps;
    motor.setVoltage(-rps);
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