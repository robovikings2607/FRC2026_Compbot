package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;

import static edu.wpi.first.units.Units.*;

import java.security.Timestamp;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX rollerMotor;
  private final TalonFX pivotMotor;
  private final CANcoder pivotEncoder;

  private PositionVoltage control  = new PositionVoltage(0);
  private PhoenixPIDController pid = new PhoenixPIDController(0, 0, 0);

  private double intakePosition, output;
  private boolean isDeployed, isUp;

  private RobotContainer robot;

  public IntakeSubsystem(RobotContainer robot) {
    this.robot = robot;

    rollerMotor = new TalonFX(IntakeConstants.ROLLER_ID);
    pivotMotor = new TalonFX(IntakeConstants.PIVOT_ID);
    pivotEncoder = new CANcoder(IntakeConstants.ENCODER_ID);

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

    output = pid.calculate(pivotEncoder.getAbsolutePosition().getValueAsDouble(), intakePosition, Timer.getFPGATimestamp());
    pivotMotor.set(output);

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
            .withStatorCurrentLimit(Amps.of(35))
            .withStatorCurrentLimitEnable(true)
             .withSupplyCurrentLimit(Amps.of(40))
            .withSupplyCurrentLowerLimit(Amps.of(10))
            .withSupplyCurrentLimitEnable(true)
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
            .withSupplyCurrentLimit(Amps.of(40))
            .withSupplyCurrentLowerLimit(Amps.of(10))
            .withSupplyCurrentLimitEnable(true)
    );
  }

  public void runRollersUnjammed(){
    rollerMotor.setVoltage(10.5);
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
    pivotMotor.setNeutralMode(NeutralModeValue.Coast);
    intakePosition = IntakeConstants.INTAKE_DEPLOYED;
    //pivotMotor.setControl(control.withPosition(IntakeConstants.INTAKE_DEPLOYED));
  }

  public void retractIntake(){
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    intakePosition = IntakeConstants.INTAKE_RETRACTED;
    //pivotMotor.setControl(control.withPosition(IntakeConstants.INTAKE_RETRACTED));
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
}