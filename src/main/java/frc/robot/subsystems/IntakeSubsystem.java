package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;

import static edu.wpi.first.units.Units.*;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX rollerMotor;
  private final TalonFX pivotMotor;

  private PositionVoltage control  = new PositionVoltage(0);

  private double intakePosition;
  private boolean isDeployed, isUp;

  private RobotContainer robot;

  public IntakeSubsystem(RobotContainer robot) {
    this.robot = robot;

    rollerMotor = new TalonFX(IntakeConstants.ROLLER_ID);
    pivotMotor = new TalonFX(IntakeConstants.PIVOT_ID);

    intakePosition = IntakeConstants.INTAKE_RETRACTED;
    isDeployed = false;
    isUp = false;

    configurePivotMotor();
    configureRollerMotor();
    // zeroIntake();
  }

  @Override
  public void periodic() {

/*     if(isDeployed){
      pivotMotor.setVoltage(-0.2);
      pivotMotor.setNeutralMode(NeutralModeValue.Coast);
      isUp = false;
    }
    else{
      if(isUp){
        pivotMotor.setVoltage(0.5);
      }
      else{
        pivotMotor.setVoltage(4.0);
        isUp = pivotMotor.getStatorCurrent().getValueAsDouble() > 60;
      }      
    }
     */

   /*  if(RobotController.getUserButton()){
      pivotMotor.setPosition(0.0);
    } */

    SmartDashboard.putBoolean("Intake/IsJammed", isJammed());
    SmartDashboard.putNumber("Intake/Position", pivotMotor.getPosition().getValueAsDouble());
  }

  public void configurePivotMotor(){
    TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.Slot0.kP = 2.5; // An error of 1 rotation results in 2.4 V output
        configs.Slot0.kI = 0; // No output for integrated error
        configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output 

    configs.withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(50))
            .withStatorCurrentLimitEnable(true)
    );

    pivotMotor.getConfigurator().apply(configs);
    //ppivotMotor.setPosition(0.0);
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    pivotMotor.setControl(control.withPosition(IntakeConstants.INTAKE_RETRACTED));
  }

  public void configureRollerMotor(){
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.withCurrentLimits(
        new CurrentLimitsConfigs()
            // Swerve azimuth does not require much torque output, so we can set a relatively low
            // stator current limit to help avoid brownouts without impacting performance.
            .withStatorCurrentLimit(Amps.of(120))
            .withStatorCurrentLimitEnable(true)
    );
  }

  public void runRollersUnjammed(){
    rollerMotor.setVoltage(9.0);
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

  public void deployIntake(double currentPos){
    pivotMotor.setNeutralMode(NeutralModeValue.Coast);
    pivotMotor.setControl(control.withPosition(currentPos));
  }

  public void retractIntake(double currentPos){
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    pivotMotor.setControl(control.withPosition(currentPos));
  }

 /*  public void zeroIntake(){
    pivotMotor.set(0.5);

    if(pivotMotor.getStatorCurrent().getValueAsDouble() > 10.0){
      pivotMotor.set(0.0);
      pivotMotor.setPosition(0.0);
    }
  } */

  public TalonFX getPivotMotor(){
    return pivotMotor;
  }

  public void forcePivotDown(){
    pivotMotor.setVoltage(2.0);
    runRollersUnjammed();
  }

  public void forcePivotUp(){
    pivotMotor.setVoltage(-2.0);
    stopRollers();
  }

  public boolean downOrUp(){
    return pivotMotor.getStatorCurrent().getValueAsDouble() > 40.0;
  }
}