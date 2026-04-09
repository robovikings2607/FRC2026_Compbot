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
import frc.robot.utilities.RobotLogger;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RollerConstants;

import static edu.wpi.first.units.Units.*;

public class IntakeSubsystem extends SubsystemBase {

  private RobotContainer robot;
  private final TalonFX rollerMotor = new TalonFX(RollerConstants.MOTOR_ID);
  private final TalonFX pivotMotor = new TalonFX(PivotConstants.MOTOR_ID);
  private final CANcoder encoder = new CANcoder(PivotConstants.ENCODER_ID);
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
    createTuningData();
  }

  @Override
  public void periodic() {
    updateLoggingData();
    RobotLogger.logDouble("Time", Timer.getFPGATimestamp());
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
    FORCED_DOWN,
    PID_TUNING,
    JAMMED,
    UNJAM
  }

  public IntakeState getState(){
    return state;
  }

  public enum PivotState{
    DEPLOYED,
    RETRACTED,
    FORCED_DOWN,
    OFF,
    PID_TUNING
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

  public void forceDownControl(){
    pivotMotor.setVoltage(PivotConstants.FORCE_DOWN_SPEED);
  }

  public void stopMotor(){
    pivotMotor.stopMotor();
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

  public void controlPivot(PivotState state){
    pivotState = state;
    
    switch (pivotState) {
      case RETRACTED:
        retractPivotControl();
        break;
    
      case DEPLOYED:
        deployPivotControl();
        break;

      case FORCED_DOWN:
        forceDownControl();
        if(hittingBumper()){
          pivotState = PivotState.OFF;
        }
        break;

      case OFF:
        stopMotor();
        break;

      case PID_TUNING:
        PIDTuningPivotControl();
        break;

      default:
        stopMotor();
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

  public void controlRoller(RollerState state){
    rollerState = state;

    switch (rollerState) {
      case OFF:
        stopRollers();
        break;
    
      case NORMAL:
        normalRollerControl();
        break;

      case JAMMED:
        jammedRollerControl();
        break;

      case UNJAM:
        reverseRollerControl();
        if(!isJammed()){
          rollerState = RollerState.NORMAL;
        }
        break;
    
      case REVERSE:
        reverseRollerControl();
        break;

      default:
        stopRollers();
        break;
    }
  }

  //Whole Intake Controls
  public void retractControl(){
    controlPivot(PivotState.RETRACTED);
    controlRoller(RollerState.OFF);
  }

  public void deployControl(){
    controlPivot(PivotState.DEPLOYED);
    controlRoller(RollerState.NORMAL);
  }

  public void reverseControl(){
    controlPivot(PivotState.DEPLOYED);
    controlRoller(RollerState.REVERSE);
  }

  public void forcedDownControl(){
    controlPivot(PivotState.FORCED_DOWN);
    controlRoller(RollerState.NORMAL);
  }

  public void PIDTuningControl(){
    controlPivot(PivotState.PID_TUNING);
    controlRoller(RollerState.OFF);
  }

  public void jammedControl(){
    controlPivot(PivotState.DEPLOYED);
    controlRoller(RollerState.JAMMED);
  }  
  
  public void unjamControl(){
    controlPivot(PivotState.DEPLOYED);
    controlRoller(RollerState.UNJAM);
  }

  public void setState(IntakeState state){
    this.state = state;
  }

  public void controlIntake(){
    switch (state) {
      case RETRACTED:
        retractControl();
        break;

      case DEPLOYED:
        deployControl();
        if(isJammed()){
          setState(IntakeState.JAMMED);
        }
        break;

      case PID_TUNING:
        PIDTuningControl();
        break;

      case REVERSE:
        reverseControl();
        break;

      case FORCED_DOWN:
        forceDownControl();
        break;
    
      case JAMMED:
        jammedControl();
        if(!isJammed()){
          setState(IntakeState.DEPLOYED);
        }
        else if(timer.get() > 1.0){
          setState(IntakeState.UNJAM);
        }
        break;

      case UNJAM:
        if(!isJammed()){
          setState(IntakeState.DEPLOYED);
        }
        break;

      default:
        retractControl();
        break;
    }
  }

  public void updateLoggingData(){
    RobotLogger.logString("Intake/State", state.name());
    //Pivot
    RobotLogger.logDouble("Intake/Pivot/Output", output);
    RobotLogger.logDouble("Intake/Pivot/CurrentPosition", encoder.getAbsolutePosition().getValueAsDouble());
    RobotLogger.logDouble("Intake/Pivot/Goal", pid.getSetpoint());
    RobotLogger.logBoolean("Intake/Pivot/AtGoal", pid.atSetpoint());
    RobotLogger.logDouble("Intake/Pivot/StatorCurrent", pivotMotor.getStatorCurrent().getValueAsDouble());
    RobotLogger.logDouble("Intake/Pivot/SupplyCurrent", pivotMotor.getSupplyCurrent().getValueAsDouble());
    RobotLogger.logDouble("Intake/Pivot/Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());
    RobotLogger.logString("Intake/Pivot/State", pivotState.name());
    //Roller
    RobotLogger.logDouble("Intake/Roller/StatorCurrent", rollerMotor.getStatorCurrent().getValueAsDouble());
    RobotLogger.logDouble("Intake/Roller/SupplyCurrent", rollerMotor.getSupplyCurrent().getValueAsDouble());
    RobotLogger.logDouble("Intake/Roller/Voltage", rollerMotor.getMotorVoltage().getValueAsDouble());
    RobotLogger.logString("Intake/Roller/State", rollerState.name());
  }

  public void createTuningData(){
    RobotLogger.logDouble("Intake/Pivot/Tuning/PID/P", 0);
    RobotLogger.logDouble("Intake/Pivot/Tuning/PID/I", 0);
    RobotLogger.logDouble("Intake/Pivot/Tuning/PID/D", 0);
    RobotLogger.logBoolean("Intake/Pivot/Tuning/Deploy?", false);
  }

  public boolean isJammed(){
    return rollerMotor.getStatorCurrent().getValueAsDouble() > RollerConstants.JAMMED_THRESHOLD;
  }

  public boolean hittingBumper(){
    return pivotMotor.getStatorCurrent().getValueAsDouble() > PivotConstants.FORCED_DOWN_THRESHOLD;
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