package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX rollerMotor;
  private final TalonFX pivotMotor;

  private final PositionVoltage positionVoltage = new PositionVoltage(0);

  private double intakePosition = IntakeConstants.INTAKE_RETRACTED;

  private RobotContainer robot;

  public IntakeSubsystem(RobotContainer robot) {
    this.robot = robot;

    rollerMotor = new TalonFX(IntakeConstants.ROLLER_ID);
    pivotMotor = new TalonFX(IntakeConstants.PIVOT_ID);

    configurePivotMotor();
    zeroIntake();
  }

  @Override
  public void periodic() {
    pivotMotor.setControl(positionVoltage.withPosition(intakePosition));
  }

  public void configurePivotMotor(){
    TalonFXConfiguration configs = new TalonFXConfiguration();

    var slot0Configs = configs.Slot0;
        slot0Configs.kS = 0.0; // Voltage output to overcome static friction
        slot0Configs.kV = 0.0; // A velocity target of 1 rps requires this voltage output.
        slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires this voltage output
        slot0Configs.kP = 0.0; // A position error of 2.5 rotations requires this voltage output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.0; // A velocity error of 1 rps requires this voltage output
    
    pivotMotor.getConfigurator().apply(slot0Configs);
  }

  public void runRollers(){
    if(!isJammed()){
      rollerMotor.set(0.5);
    }
    else{
      rollerMotor.set(1.0);
    }
  }

  public void stopRollers(){
    rollerMotor.set(0.0);
  }

  public boolean isJammed(){
    return rollerMotor.getStatorCurrent().getValueAsDouble() > 50.0;
  }

  public void deployIntake(){
    intakePosition = IntakeConstants.INTAKE_DEPLOYED;
  }

  public void retractIntake(){
    intakePosition = IntakeConstants.INTAKE_RETRACTED;
  }

  public void zeroIntake(){
    pivotMotor.set(0.5);

    if(pivotMotor.getStatorCurrent().getValueAsDouble() > 10.0){
      pivotMotor.set(0.0);
      pivotMotor.setPosition(0.0);
    }
  }
}