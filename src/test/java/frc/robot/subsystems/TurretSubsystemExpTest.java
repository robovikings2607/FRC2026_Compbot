package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utilities.GeometryUtil;

public class TurretSubsystemExpTest {

    // --- A Helper Method for Clean Assertions ---
    // Because floating point math (doubles) can have tiny rounding errors, 
    // we use a small delta (0.001) when comparing coordinates.


// ... inside your test class ...

    private void assertPoseEquals(Pose2d expected, Pose2d actual, String message) {
        double delta = 0.001;
        
        // Check X and Y normally
        assertEquals(expected.getX(), actual.getX(), delta, message + " (X mismatch)");
        assertEquals(expected.getY(), actual.getY(), delta, message + " (Y mismatch)");
        
        // Check Angle with wrapping
        double expectedDeg = expected.getRotation().getDegrees();
        double actualDeg = actual.getRotation().getDegrees();
        
        // inputModulus wraps the error to be between -180 and 180. 
        // 270 and -90 will have an error of 360, which wraps to 0.
        double angleError = Math.abs(MathUtil.inputModulus(expectedDeg - actualDeg, -180.0, 180.0));
        
        assertEquals(0.0, angleError, delta, message + " (Angle mismatch: expected " 
                    + expectedDeg + " but got " + actualDeg + ")");
    }


}