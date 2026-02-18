package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

public class ShooterState implements Interpolatable<ShooterState> {
    public final double rps;
    public final double hoodAngle;

    public ShooterState(double rps, double hoodAngle) {
        this.rps = rps;
        this.hoodAngle = hoodAngle;
    }

    @Override
    public ShooterState interpolate(ShooterState endValue, double t) {
        // 't' is a percentage (0.0 to 1.0) of how far we are between the two known points.
        // MathUtil handles the linear interpolation for both variables automatically.
        return new ShooterState(
            MathUtil.interpolate(this.rps, endValue.rps, t),
            MathUtil.interpolate(this.hoodAngle, endValue.hoodAngle, t)
        );
    }
}