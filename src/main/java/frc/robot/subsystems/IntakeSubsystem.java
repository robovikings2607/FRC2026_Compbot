package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RollerConstants;

import static edu.wpi.first.units.Units.*;

public class IntakeSubsystem extends SubsystemBase {

  private RobotContainer robot;
  private final TalonFX rollerMotor = new TalonFX(RollerConstants.MOTOR_ID);
  private final TalonFX pivotMotor = new TalonFX(PivotConstants.MOTOR_ID);
  private final CANcoder encoder = new CANcoder(PivotConstants.MOTOR_ID);
  private final PhoenixPIDController pid = new PhoenixPIDController(PivotConstants.P, PivotConstants.I, PivotConstants.D);
  private final Timer timer = new Timer();
  private double output;
  private IntakeState state = IntakeState.RETRACTED;
  private PivotState pivotState = PivotState.RETRACTED;
  private RollerState rollerState = RollerState.OFF;

  public IntakeSubsystem(RobotContainer robot) {
    this.robot = robot;

    configurePivotMotor();
    configureRollerMotor();
    //configureEncoder();
    //configurePID();
  }

  @Override
  public void periodic() {
    updateLoggingData();
  }

  public void configurePivotMotor(){
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(PivotConstants.STATOR_LIMIT))
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(PivotConstants.SUPPLY_LIMIT))
            .withSupplyCurrentLowerLimit(Amps.of(PivotConstants.SUPPLY_LOWER_LIMIT))
            .withSupplyCurrentLimitEnable(true)
    );

    pivotMotor.getConfigurator().apply(configs);
  }

  public void configureRollerMotor(){
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.withCurrentLimits(
        new CurrentLimitsConfigs()
            // Swerve azimuth does not require much torque output, so we can set a relatively low
            // stator current limit to help avoid brownouts without impacting performance.
            .withStatorCurrentLimit(Amps.of(RollerConstants.STATOR_LIMIT))
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(RollerConstants.SUPPLY_LIMIT))
            .withSupplyCurrentLowerLimit(Amps.of(RollerConstants.SUPPLY_LOWER_LIMIT))
            .withSupplyCurrentLimitEnable(true)
    );
  }

  public void configureEncoder(){}

  public void configurePID(){}

  public enum IntakeState{
    DEPLOYED,
    RETRACTED,
    REVERSE,
    PID_TUNING
  }

  public void setState(IntakeState state){
    this.state = state;
  }

  public IntakeState getState(){
    return state;
  }

  public enum PivotState{
    DEPLOYED,
    RETRACTED,
    PID_TUNING
  }
  
  public void setPivotState(PivotState state){
    pivotState = state;
  }

  public PivotState getPivotState(){
    return pivotState;
  }

  public enum RollerState{
    NORMAL,
    JAMMED,
    REVERSE,
    UNJAM,
    OFF
  }

  public void setRollerState(RollerState state){
    rollerState = state;
  }

  public RollerState getRollerState(){
    return rollerState;
  }

  //Pivot Controls
  public void deployPivotControl(){
    output = pid.calculate(encoder.getAbsolutePosition().getValueAsDouble(), PivotConstants.DEPLOYED, Timer.getFPGATimestamp());
    pivotMotor.set(output);
  }  
  
  public void retractPivotControl(){
    output = pid.calculate(encoder.getAbsolutePosition().getValueAsDouble(), PivotConstants.RETRACTED, Timer.getFPGATimestamp());
    pivotMotor.set(output);
  }

  public void PIDTuningPivotControl(){
    pid.setP(SmartDashboard.getNumber("Intake/Pivot/Tuning/PID/P", 0));
    pid.setI(SmartDashboard.getNumber("Intake/Pivot/Tuning/PID/I", 0));
    pid.setD(SmartDashboard.getNumber("Intake/Pivot/Tuning/PID/D", 0));
    if(SmartDashboard.getBoolean("Intake/Pivot/Tuning/Deploy?", false)){
      output = pid.calculate(encoder.getAbsolutePosition().getValueAsDouble(), PivotConstants.DEPLOYED, Timer.getFPGATimestamp());
    }
    else{
      output = pid.calculate(encoder.getAbsolutePosition().getValueAsDouble(), PivotConstants.RETRACTED, Timer.getFPGATimestamp());
    }
    pivotMotor.set(output);    
  }

  public void controlPivot(){
    switch (pivotState) {
      case RETRACTED:
        retractPivotControl();
        break;
    
      case DEPLOYED:
        deployPivotControl();
        break;

      default:
        retractPivotControl();
        break;
    }
  }

  //Roller Controls
  public void normalRollerControl(){
    rollerMotor.setVoltage(RollerConstants.NORMAL_SPEED);
  }

  public void jammedRollerControl(){
    rollerMotor.setVoltage(RollerConstants.JAMMED_SPEED);
    timer.restart();
  }

  public void reverseRollerControl(){
    rollerMotor.setVoltage(-RollerConstants.NORMAL_SPEED);
    timer.reset();
  }

  public void stopRollers(){
    rollerMotor.stopMotor();;
  }

  public void controlRoller(){
    switch (rollerState) {
      case OFF:
        stopRollers();
        break;
    
      case NORMAL:
        normalRollerControl();
        if(isJammed()){
          rollerState = RollerState.JAMMED;
        }
        break;

      case JAMMED:
        jammedRollerControl();
        if(!isJammed()){
          rollerState = RollerState.NORMAL;
        }
        else if(timer.get() > 1.0){
          rollerState = RollerState.UNJAM;
        }
        break;

      case UNJAM:
        reverseControl();
        if(!isJammed()){
          rollerState = RollerState.NORMAL;
        }
        break;
    
      case REVERSE:
        reverseControl();
        break;

      default:
        stopRollers();
        break;
    }
  }

  //Whole Intake Controls
  public void retractControl(){
    setPivotState(PivotState.RETRACTED);
    setRollerState(RollerState.OFF);
    controlPivot();
    controlRoller();
  }

  public void deployControl(){
    setPivotState(PivotState.DEPLOYED);
    setRollerState(RollerState.NORMAL);
    controlPivot();
    controlRoller();
  }

  public void reverseControl(){
    setPivotState(PivotState.DEPLOYED);
    setRollerState(RollerState.REVERSE);
    controlPivot();
    controlRoller();
  }


  public void PIDTuningControl(){
    setPivotState(PivotState.PID_TUNING);
    setRollerState(RollerState.OFF);
    controlPivot();
    controlRoller();
  }

  public void controlIntake(){
    switch (state) {
      case RETRACTED:
        retractControl();
        break;

      case DEPLOYED:
        deployControl();
        break;

      case PID_TUNING:
        PIDTuningControl();
        break;

      case REVERSE:
        reverseControl();
        break;
    
      default:
        retractControl();
        break;
    }
  }

  public void updateLoggingData(){
    SmartDashboard.putString("Intake/State", state.name());
    //Pivot
    SmartDashboard.putNumber("Intake/Pivot/Output", output);
    SmartDashboard.putNumber("Intake/Pivot/CurrentPosition", encoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/Goal", pid.getSetpoint());
    SmartDashboard.putBoolean("Intake/Pivot/AtGoal", pid.atSetpoint());
    SmartDashboard.putNumber("Intake/Pivot/StatorCurrent", pivotMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/SupplyCurrent", pivotMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putString("Intake/Pivot/State", pivotState.name());
    //Roller
    SmartDashboard.putNumber("Intake/Roller/StatorCurrent", rollerMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Roller/SupplyCurrent", rollerMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Roller/Voltage", rollerMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putString("Intake/Roller/State", rollerState.name());
  }

  public void createTuningData(){
    SmartDashboard.putNumber("Intake/Pivot/Tuning/PID/P", 0);
    SmartDashboard.putNumber("Intake/Pivot/Tuning/PID/I", 0);
    SmartDashboard.putNumber("Intake/Pivot/Tuning/PID/D", 0);
    SmartDashboard.putBoolean("Intake/Pivot/Tuning/Deploy?", false);
  }

  public boolean isJammed(){
    return rollerMotor.getStatorCurrent().getValueAsDouble() > RollerConstants.JAMMED_THRESHOLD;
  }

  public TalonFX getPivotMotor(){
    return pivotMotor;
  }

  public TalonFX getRollerMotor(){
    return rollerMotor;
  }

  public CANcoder getEncoder(){
    return encoder;
  }
}