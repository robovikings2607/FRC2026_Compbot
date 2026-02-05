package frc.robot.utilities;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AxisButton extends Trigger{
    public AxisButton(XboxController js, int axisID, double threshold)
    {
        super(() -> {
            return Math.abs(js.getRawAxis(axisID)) > threshold;
        });
    }
}
