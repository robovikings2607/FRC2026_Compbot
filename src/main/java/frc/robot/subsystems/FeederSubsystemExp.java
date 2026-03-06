package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.utilities.SysIdBuilder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.FeederConstants;


public class FeederSubsystemExp extends SubsystemBase {
    private TalonFX motor = new TalonFX(FeederConstants.FEEDER_ID);
    private final VelocityVoltage velocityReq = new VelocityVoltage(0).withSlot(0);

    private final double FEEDER_TARGET_RPS = 30.0; 

    private final SysIdRoutine sysIdRoutine = SysIdBuilder.buildTalonFXRoutine(
        motor, this, "feeder", 4.0
    );    

    public FeederSubsystemExp(RobotContainer robot) {
      configureMotor();
    }

    public SysIdRoutine getSysIdRoutine() {
        return sysIdRoutine;
    }    

    public void runForwardAtTunedSpeed() {
        motor.setControl(velocityReq.withVelocity(FEEDER_TARGET_RPS));
    }

    public void runGentleReverse() {
        // We can just use raw voltage/percent output for unjamming
        // since we just need a quick, generic backward tug.
        motor.set(-0.30); 
    }

    public void stop() {
        motor.setControl(velocityReq.withVelocity(0));
    }

  public void configureMotor(){

    TalonFXConfiguration configs = new TalonFXConfiguration();

    var slot0Configs = configs.Slot0;
        slot0Configs.kS = 0.25; // Voltage output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps requires this voltage output.
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires this voltage output
        slot0Configs.kP = 4.8; // A position error of x rotations requires this voltage output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.11; // A velocity error of 1 rps requires this voltage output

    motor.getConfigurator().apply(configs);
    motor.setNeutralMode(NeutralModeValue.Brake);

    motor.setPosition(0);
  }

}