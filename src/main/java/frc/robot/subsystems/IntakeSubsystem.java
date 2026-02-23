package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX rollerMotor;
  private final TalonFX pivotMotor;

  private MotionMagicVoltage control  = new MotionMagicVoltage(0);

  private double intakePosition;
  private boolean isDeployed;

  private RobotContainer robot;

  public IntakeSubsystem(RobotContainer robot) {
    this.robot = robot;

    rollerMotor = new TalonFX(IntakeConstants.ROLLER_ID);
    pivotMotor = new TalonFX(IntakeConstants.PIVOT_ID);

    intakePosition = IntakeConstants.INTAKE_RETRACTED;
    isDeployed = false;

    configurePivotMotor();
    // zeroIntake();
  }

  @Override
  public void periodic() {

    pivotMotor.setControl(control.withPosition(intakePosition));
    SmartDashboard.putBoolean("Intake/IsJammed", isJammed());
  }

  public void configurePivotMotor(){
    TalonFXConfiguration configs = new TalonFXConfiguration();

     var slot0Configs = configs.Slot0;
        slot0Configs.kS = 0.25; // Voltage output to overcome static friction
        slot0Configs.kV = 3.0; // A velocity target of 1 rps requires this voltage output.
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires this voltage output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations requires this voltage output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.11; // A velocity error of 1 rps requires this voltage output

    var motionMagicConfigs = configs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
    
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    pivotMotor.getConfigurator().apply(slot0Configs);
    pivotMotor.setPosition(0.0);
  }

  public void runRollersUnjammed(){
    rollerMotor.setVoltage(8.0);
  }

  public void runRollersJammed(){
    rollerMotor.setVoltage(12.0);
  }

  public void stopRollers(){
    rollerMotor.setVoltage(0.0);
  }

  public void reverseRollers(){
    rollerMotor.setVoltage(-8.0);
  }

  public boolean isJammed(){
    return rollerMotor.getStatorCurrent().getValueAsDouble() > 100.0;
  }

  public void deployIntake(){
    intakePosition = IntakeConstants.INTAKE_DEPLOYED;
    isDeployed = true;
  }

  public void retractIntake(){
    intakePosition = IntakeConstants.INTAKE_RETRACTED;
    isDeployed = false;
  }

 /*  public void zeroIntake(){
    pivotMotor.set(0.5);

    if(pivotMotor.getStatorCurrent().getValueAsDouble() > 10.0){
      pivotMotor.set(0.0);
      pivotMotor.setPosition(0.0);
    }
  } */
}