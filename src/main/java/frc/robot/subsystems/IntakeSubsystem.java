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

  private PositionVoltage control  = new PositionVoltage(0);

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
    SmartDashboard.putNumber("Intake/Position", intakePosition);
  }

  public void configurePivotMotor(){
    TalonFXConfiguration configs = new TalonFXConfiguration();

     var slot0Configs = configs.Slot0;
          configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
          configs.Slot0.kI = 0; // No output for integrated error
          configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    pivotMotor.getConfigurator().apply(slot0Configs);
    pivotMotor.setPosition(0.0);
  }

  public void runRollersUnjammed(){
    rollerMotor.setVoltage(10.0);
  }

  public void runRollersJammed(){
    rollerMotor.setVoltage(12.0);
  }

  public void stopRollers(){
    rollerMotor.setVoltage(0.0);
  }

  public void reverseRollers(){
    rollerMotor.setVoltage(-6.0);
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