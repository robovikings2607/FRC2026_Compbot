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
    private final StatusSignal<Angle> motorPositionSignal;  
    private final StatusSignal<AngularVelocity> motorVelocitySignal;    
    private final StatusSignal<Voltage> motorVoltageSignal;      
    private final StatusSignal<Temperature> motorTemperatureSignal;      

    public MotorUtil(TalonFX motor) {
    
        this.motor = motor;

        statorCurrentSignal = this.motor.getStatorCurrent();
        statorCurrentSignal.setUpdateFrequency(50);

        supplyCurrentSignal = this.motor.getSupplyCurrent();
        supplyCurrentSignal.setUpdateFrequency(50);

        motorPositionSignal = this.motor.getPosition();
        motorPositionSignal.setUpdateFrequency(50);

        motorVelocitySignal = this.motor.getVelocity();
        motorVelocitySignal.setUpdateFrequency(50);

        motorVoltageSignal = this.motor.getMotorVoltage();
        motorVoltageSignal.setUpdateFrequency(50);

        motorTemperatureSignal = this.motor.getDeviceTemp();
        motorTemperatureSignal.setUpdateFrequency(50);
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
        return motorPositionSignal.refresh().getValue();    
    }

    /**
     * Retrieves the velocity of the motor
     */
    public AngularVelocity getVelocity() {
        return motorVelocitySignal.refresh().getValue();    
    }

    /**
     * Retrieves the voltage being applied to the motor
     */
    public Voltage getMotorVoltage() {
        return motorVoltageSignal.refresh().getValue();    
    }

        /**
     * Retrieves the temperature being applied to the motor
     */
    public Temperature getMotorTemperature() {
        return motorTemperatureSignal.refresh().getValue();    
    }
}
