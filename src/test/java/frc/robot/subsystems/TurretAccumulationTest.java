package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

public class TurretAccumulationTest {

    // Copied from TurretSubsystem / Constants
    private static final double rotationsPerDegree = 10.0 / 360.0;
    private static final double WRAP_THRESHOLD     = 9.5;
    private static final double WRAP_PERIOD        = 10.0;

    // From Constants.java (OFFSET = 0)
    private static final double MAX_ANGLE_DEG = 241.2 - 10.0;            // 231.2 degrees
    private static final double MIN_ANGLE_DEG = -137.988 + 10.0;         // -127.988 degrees
    private static final double MAX_ROTATIONS  = MAX_ANGLE_DEG * rotationsPerDegree; // ~6.42
    private static final double MIN_ROTATIONS  = MIN_ANGLE_DEG * rotationsPerDegree; // ~-3.55

    /** Exact copy of TurretSubsystem.getDelta */
    private static double getDelta(double previousSetPoint, double newSetPoint) {
        double delta = 0;
        if (Math.abs(previousSetPoint - newSetPoint) > WRAP_THRESHOLD) {
            delta = WRAP_PERIOD - Math.abs(previousSetPoint - newSetPoint);
            if (previousSetPoint < newSetPoint) {
                delta = -delta;
            }
        } else {
            delta = newSetPoint - previousSetPoint;
        }
        return delta;
    }

    /** Exact copy of the clamp block in TurretSubsystem.periodic */
    private static double clamp(double newEncoderPos) {
        if (newEncoderPos > MAX_ROTATIONS) {
            newEncoderPos -= 360 * rotationsPerDegree;
        } else if (newEncoderPos < MIN_ROTATIONS) {
            newEncoderPos += 360 * rotationsPerDegree;
        }
        return newEncoderPos;
    }

    @Test
    public void testNormalTrackingNearMinAngle() {
        // Turret tracking normally just inside the MIN limit
        double previousSetPoint   = -3.4;
        double previousEncoderPos = -3.4;

        double newSetPoint    = -3.5;
        double delta          = getDelta(previousSetPoint, newSetPoint);
        double newEncoderPos  = clamp(previousEncoderPos + delta);

        // -3.5 is still above MIN_ROTATIONS (-3.555), so no clamp fires
        assertTrue(newEncoderPos > MIN_ROTATIONS,
            "Should stay within bounds, newEncoderPos=" + newEncoderPos);
        assertEquals(-3.5, newEncoderPos, 0.001,
            "Encoder should match setpoint during normal tracking");
    }

    @Test
    public void testSetpointCrossesMinTriggersFalseWrap() {
        // Turret tracking just inside the MIN limit
        double previousSetPoint   = -3.4;
        double previousEncoderPos = -3.4;

        // Robot turns slightly so the field-geometry setpoint drops below MIN_ROTATIONS
        double newSetPoint = -3.7;  // below MIN_ROTATIONS (-3.555)

        double delta         = getDelta(previousSetPoint, newSetPoint);
        double newEncoderPos = clamp(previousEncoderPos + delta);

        // getDelta sees a small (-0.3) move, no wrap detection
        assertEquals(-0.3, delta, 0.001, "Delta should be a small, normal move");

        // But the clamp code fires and adds 10 rotations — turret jumps to opposite extreme
        double expected = -3.7 + (WRAP_PERIOD); // ~6.3
        assertEquals(expected, newEncoderPos, 0.001,
            "Clamp wraps newEncoderPos to the opposite extreme (~6.3 rotations)");

        assertTrue(newEncoderPos > MAX_ROTATIONS * 0.9,
            "Motor is now commanded to the far end of its travel, near MAX soft limit");


        // Pick up from the broken state one loop in the future: clamp fired, previousEncoderPos is now at ~6.3
        previousSetPoint   = newSetPoint;
        previousEncoderPos = newEncoderPos;

        // Field geometry continues to give setpoints near -3.7 to -4.0
        double[] subsequentSetPoints = { -3.75, -3.80, -3.85, -3.72 };

        for (double sp : subsequentSetPoints) {
            delta         = getDelta(previousSetPoint, sp);
            newEncoderPos = clamp(previousEncoderPos + delta);

            // delta is always tiny (< 0.2), no wrap triggered
            assertTrue(Math.abs(delta) < 0.2, "Delta should remain small");

            // Motor is commanded near MAX (6.3 ± small), not near MIN (~-3.5) where it should be
            assertTrue(newEncoderPos > 5.0,
                "Motor remains commanded at wrong extreme (~" + newEncoderPos
                + ") instead of near " + sp);

            // The correct position and commanded position are ~10 rotations apart — no recovery
            double error = Math.abs(newEncoderPos - sp);
            assertTrue(error > 9.0,
                "Error of " + error + " rotations persists with no recovery mechanism");

            previousSetPoint   = sp;
            previousEncoderPos = newEncoderPos;
        }
    }
}
