package frc.robot.utilities;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class MotorUtil {
    private final TalonFX motor;    
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Current> supplyCurrentSignal;    
    private final StatusSignal<Angle> positionSignal;  
    private final StatusSignal<AngularVelocity> velocitySignal;    
    private final StatusSignal<Voltage> voltageSignal;      
    private final StatusSignal<Temperature> temperatureSignal;      
    private final StatusSignal<Double> closedLoopErrorSignal;          

    public MotorUtil(TalonFX motor) {
    
        this.motor = motor;

        statorCurrentSignal = this.motor.getStatorCurrent();
        statorCurrentSignal.setUpdateFrequency(50);

        supplyCurrentSignal = this.motor.getSupplyCurrent();
        supplyCurrentSignal.setUpdateFrequency(50);

        positionSignal = this.motor.getPosition();
        positionSignal.setUpdateFrequency(50);

        velocitySignal = this.motor.getVelocity();
        velocitySignal.setUpdateFrequency(50);

        voltageSignal = this.motor.getMotorVoltage();
        voltageSignal.setUpdateFrequency(50);

        temperatureSignal = this.motor.getDeviceTemp();
        temperatureSignal.setUpdateFrequency(50);

        closedLoopErrorSignal = this.motor.getClosedLoopError();
        closedLoopErrorSignal.setUpdateFrequency(50);
    }

    /**
     * Retrieves the stator current
     */
    public Current getStatorCurrent() {
        return statorCurrentSignal.refresh().getValue();    
    }

    /**
     * Retrieves the supply current
     */
    public Current getSupplyCurrent() {
        return supplyCurrentSignal.refresh().getValue();    
    }

    /**
     * Retrieves the position of the mechanism controlled by the motor
     */
    public Angle getPosition() {
        return positionSignal.refresh().getValue();    
    }

    /**
     * Retrieves the velocity of the motor
     */
    public AngularVelocity getVelocity() {
        return velocitySignal.refresh().getValue();    
    }

    /**
     * Retrieves the voltage being applied to the motor
     */
    public Voltage getMotorVoltage() {
        return voltageSignal.refresh().getValue();    
    }

    /**
     * Retrieves the temperature being applied to the motor
     */
    public Temperature getMotorTemperature() {
        return temperatureSignal.refresh().getValue();    
    }

    /**
     * Retrieves the closed loop error of the motor
     */
    public Double getClosedLoopError() {
        return closedLoopErrorSignal.refresh().getValue();    
    }

}
