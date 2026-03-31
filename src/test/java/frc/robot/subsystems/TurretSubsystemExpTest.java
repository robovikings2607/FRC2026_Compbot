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

    @Test
    public void testRobot350TargetMinus150Degrees() {
        System.out.println("DEBUGGING TURRET:");   
        
        // 1. The Robot Pose
        // Placed at the center of the field, facing slightly right of center (350 degrees)
        Pose2d robotPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(350.0));

        // 2. The Target Pose
        // Placed 5 meters away at an absolute field angle of -150 degrees (bottom left quadrant)
        double distance = 5.0;
        double targetX = distance * Math.cos(Math.toRadians(-150.0)); // Approx -4.33
        double targetY = distance * Math.sin(Math.toRadians(-150.0)); // Approx -2.50
        Pose2d targetPose = new Pose2d(targetX, targetY, new Rotation2d());

        double minAngleDegrees = -127.988;
        double maxAngleDegrees = 231.2;

        double turretAngle = TurretSubsystemExp.calcUnclampedTurretAngleToTargetDegrees(
            targetPose.getTranslation(), 
            robotPose,
            minAngleDegrees,
            maxAngleDegrees
        );

        System.out.println("Turret Angle: " + turretAngle);
        assertEquals(221.5, turretAngle, 0.1);

    }

}