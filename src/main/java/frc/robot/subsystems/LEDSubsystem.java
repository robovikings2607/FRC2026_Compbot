package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LedSubsystem. */
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  public RobotContainer m_robot;
  private LEDPattern currentPattern = LEDPattern.solid(Color.kBlack);

  public LEDSubsystem(RobotContainer robot) {
    m_robot = robot;
    
    // PWM port 0
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(0);
    

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(92);
    m_led.setLength(m_ledBuffer.getLength());
    // Create an LED pattern that sets the entire strip to solid red
    LEDPattern red = LEDPattern.solid(Color.kBlack);

    // Apply the LED pattern to the data buffer
    red.applyTo(m_ledBuffer);

    // Write the data to the LED strip
    m_led.setData(m_ledBuffer);
    m_led.start();

  }

  private int count = 0;
  private boolean isLEDOn = false;
  private double brightness = 40;

  @Override
  public void periodic() {
    setModeColor();
  
  }

  private void setModeColor(){
   
  }

  private void setlevelcolor() {
    
  }

  public void setColor(Color color){
      currentPattern = LEDPattern.solid(color).atBrightness(Percent.of(brightness));
  }

  private void blinking() {
    if (count < 15) {
      count++; // increment counter
    } else {
      count = 0;
      if (isLEDOn) {
        LEDPattern black = LEDPattern.solid(Color.kBlack);
        black.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
        isLEDOn = false;
      }

      else {

        currentPattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
        isLEDOn = true;

      }
    }

  }
}