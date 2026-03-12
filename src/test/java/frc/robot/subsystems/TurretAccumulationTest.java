package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import frc.robot.Constants.TurretConstants;

public class TurretAccumulationTest {

    private static final double rotationsPerDegree = 10.0 / 360.0;
    private static final double MAX_ROTATIONS = TurretConstants.MAX_ANGLE * rotationsPerDegree;
    private static final double MIN_ROTATIONS = TurretConstants.MIN_ANGLE * rotationsPerDegree;

    @Test
    public void testNormalTrackingNearMinAngle() {
        // Turret tracking normally just inside the MIN limit
        double previousSetPoint   = -3.4;
        double previousEncoderPos = -3.4;

        double newSetPoint    = -3.5;
        double delta          = TurretSubsystem.getDelta(previousSetPoint, newSetPoint);
        double newEncoderPos  = TurretSubsystem.clampEncoderPos(previousEncoderPos + delta);

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

        double delta         = TurretSubsystem.getDelta(previousSetPoint, newSetPoint);
        double newEncoderPos = TurretSubsystem.clampEncoderPos(previousEncoderPos + delta);

        // getDelta sees a small (-0.3) move, no wrap detection
        assertEquals(-0.3, delta, 0.001, "Delta should be a small, normal move");

        // But the clamp code fires and adds 10 rotations — turret jumps to opposite extreme
        double expected = -3.7 + (10.0); // ~6.3
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
            delta         = TurretSubsystem.getDelta(previousSetPoint, sp);
            newEncoderPos = TurretSubsystem.clampEncoderPos(previousEncoderPos + delta);

            // delta is always tiny (< 0.2), no wrap triggered
            assertTrue(Math.abs(delta) < 0.2, "Delta should remain small");

            // Motor is commanded near MAX (6.3 ± small), not near MIN (~-3.5) where it should be
            assertTrue(newEncoderPos > 5.0,
                "Motor remains commanded at wrong extreme (~" + newEncoderPos
                + ") instead of near " + sp);

            // The correct position and commanded position are ~10 rotations apart — no recovery
            double error = Math.abs(newEncoderPos - sp);
            assertTrue(error < 0.5,
                "Error of " + error + " rotations persists with no recovery mechanism");

            previousSetPoint   = sp;
            previousEncoderPos = newEncoderPos;
        }
    }
}
