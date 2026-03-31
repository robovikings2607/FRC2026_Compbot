// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.PortUnreachableException;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utilities.ISysIdTunable;
import frc.robot.utilities.SysIdBuilder;
import frc.robot.Constants.SpindexerConstants;

import static edu.wpi.first.units.Units.*;

public class SpindexerSubsystem extends SubsystemBase implements ISysIdTunable {
  private int reverse;
  /** Creates a new SpinDexerSubSystem. */
 private TalonFX spindexerMotor;
  public SpindexerSubsystem(RobotContainer robot) {
    spindexerMotor = new TalonFX(SpindexerConstants.SPINDEXER_ID);
  }

  private final SysIdRoutine sysIdRoutine = SysIdBuilder.buildTalonFXRoutine(
      spindexerMotor, this, "spindexer", 4.5
  );    

   public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void configureMotor(){
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.withCurrentLimits(
        new CurrentLimitsConfigs()
            // Swerve azimuth does not require much torque output, so we can set a relatively low
            // stator current limit to help avoid brownouts without impacting performance.
            .withStatorCurrentLimit(Amps.of(120))
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(40))
            .withSupplyCurrentLowerLimit(Amps.of(10))
            .withSupplyCurrentLimitEnable(true)
    );

    //TODO JJF: Get the real numbers for these values through tuning
    var slot0Configs = configs.Slot0;
        slot0Configs.kS = 0.0; // Voltage output to overcome static friction
        slot0Configs.kV = 0.0; // A velocity target of 1 rps requires this voltage output.
        slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires this voltage output
        slot0Configs.kP = 0; // A position error of x rotations requires this voltage output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.0; // A velocity error of 1 rps requires this voltage output

    spindexerMotor.getConfigurator().apply(configs);
  }

  public void runMotor() {
    spindexerMotor.setVoltage(SpindexerConstants.SPINDEXER_SPEED);
  }

  public void stopMotor() {
    spindexerMotor.setVoltage(0);
  }

  public void reverseMotor() {
    spindexerMotor.setVoltage(-SpindexerConstants.SPINDEXER_SPEED);
  }
}
