package frc.robot.utilities;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GeometryUtilTest {

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
    public void testGetTargetAngle0Degrees() {
        //System.out.println("DEBUGGING TURRET:");        
        double angle = GeometryUtil.getTargetAngleDegrees(
            new edu.wpi.first.math.geometry.Translation2d(0, 0),
            new edu.wpi.first.math.geometry.Translation2d(1, 0)
        );
        assertEquals(0.0, angle, 0.01);
    }

    @Test
    public void testGetTargetAngle180Degrees() {
        double angle = GeometryUtil.getTargetAngleDegrees(
            new edu.wpi.first.math.geometry.Translation2d(0, 0),
            new edu.wpi.first.math.geometry.Translation2d(-1, 0)
        );
        assertEquals(180.0, angle, 0.01);
    }

    @Test
    public void testGetTargetAngle90Degrees() {
        double angle = GeometryUtil.getTargetAngleDegrees(
            new edu.wpi.first.math.geometry.Translation2d(0, 0),
            new edu.wpi.first.math.geometry.Translation2d(0, 1)
        );
        assertEquals(90.0, angle, 0.01);
    }

    @Test
    public void testGetTargetAngleMinus90Degrees() {
        double angle = GeometryUtil.getTargetAngleDegrees(
            new edu.wpi.first.math.geometry.Translation2d(0, 0),
            new edu.wpi.first.math.geometry.Translation2d(0, -1)
        );
        assertEquals(-90.0, angle, 0.01);
    }

    @Test
    public void testGetTargetAngle45Degrees() {
        double angle = GeometryUtil.getTargetAngleDegrees(
            new edu.wpi.first.math.geometry.Translation2d(0, 0),
            new edu.wpi.first.math.geometry.Translation2d(1, 1)
        );
        assertEquals(45.0, angle, 0.01);
    }

    @Test
    public void testGetTargetAngleMinus45Degrees() {
        double angle = GeometryUtil.getTargetAngleDegrees(
            new edu.wpi.first.math.geometry.Translation2d(0, 0),
            new edu.wpi.first.math.geometry.Translation2d(1, -1)
        );
        assertEquals(-45.0, angle, 0.01);
    }

    @Test
    public void testGetAdjustedMechanismAngleRR30Degrees() {
        double mechanismAngleDegrees = 45.0;
        double robotAngleDegrees = 30.0;
        double ajustedMechanismAngleDegrees = GeometryUtil.getAdjustedMechanismAngleDegrees(mechanismAngleDegrees, robotAngleDegrees);
        assertEquals(15.0, ajustedMechanismAngleDegrees, 0.01);
    }

    @Test
    public void testGetAdjustedMechanismAngleRRMinusDegrees() {
        double mechanismAngleDegrees = 45.0;
        double robotAngleDegrees = -30.0;
        double ajustedMechanismAngleDegrees = GeometryUtil.getAdjustedMechanismAngleDegrees(mechanismAngleDegrees, robotAngleDegrees);
        assertEquals(75.0, ajustedMechanismAngleDegrees, 0.01);
    }

    @Test
    public void testGetAdjustedMechanismAngleM30Degrees() {
        double mechanismAngleDegrees = 30.0;
        double robotAngleDegrees = 45.0;
        double ajustedMechanismAngleDegrees = GeometryUtil.getAdjustedMechanismAngleDegrees(mechanismAngleDegrees, robotAngleDegrees);
        assertEquals(-15.0, ajustedMechanismAngleDegrees, 0.01);
    }

    @Test
    public void testGetAdjustedMechanismAngleMMinus30Degrees() {
        double mechanismAngleDegrees = -30.0;
        double robotAngleDegrees = 45.0;
        double ajustedMechanismAngleDegrees = GeometryUtil.getAdjustedMechanismAngleDegrees(mechanismAngleDegrees, robotAngleDegrees);
        assertEquals(-75.0, ajustedMechanismAngleDegrees, 0.01);
    }

    @Test
    public void testGetShortestPathToTargetDegrees30Minus30() {
        double targetDegrees = 30.0;
        double currentDegrees = -30.0;
        double shortestPathDegrees = GeometryUtil.getShortestPathToTargetDegrees(targetDegrees, currentDegrees);
        assertEquals(60.0, shortestPathDegrees, 0.01);
    }

    @Test
    public void testGetShortestPathToTargetDegreesMinus30And30() {
        double targetDegrees = -30.0;
        double currentDegrees = 30.0;
        double shortestPathDegrees = GeometryUtil.getShortestPathToTargetDegrees(targetDegrees, currentDegrees);
        assertEquals(-60.0, shortestPathDegrees, 0.01);
    }

    @Test
    public void testGetShortestPathToTargetDegrees179AndMinus179() {
        double targetDegrees = 179.0;
        double currentDegrees = -179.0;
        double shortestPathDegrees = GeometryUtil.getShortestPathToTargetDegrees(targetDegrees, currentDegrees);
        assertEquals(-2.0, shortestPathDegrees, 0.01, "actual shortestPathDegrees" + shortestPathDegrees);
    }

    @Test
    public void testGetShortestPathToTargetDegreesMinus179And179() {
        double targetDegrees = -179.0;
        double currentDegrees = 179.0;
        double shortestPathDegrees = GeometryUtil.getShortestPathToTargetDegrees(targetDegrees, currentDegrees);
        assertEquals(2.0, shortestPathDegrees, 0.01, "actual shortestPathDegrees" + shortestPathDegrees);
    }

    @Test
    public void testGetOffsetPoseRotation90() {
        Pose2d startingPoint = new Pose2d(new Translation2d(10,10), Rotation2d.fromDegrees(90));
        double offsetDistanceMeters = 2.0; 
        Rotation2d offsetAngleDegrees = new Rotation2d(0);   
        Pose2d offsetPose = GeometryUtil.getOffsetPose(startingPoint, offsetDistanceMeters, offsetAngleDegrees);    
        //System.out.println("offsetPose.getX():" + offsetPose.getX());        
        //System.out.println("offsetPose.getY():" + offsetPose.getY());                
        assertEquals(10.0, offsetPose.getX(), 0.01);
        assertEquals(12.0, offsetPose.getY(), 0.01);

    }

    public void testMotorZeroOffset180_RobotAtOrigin() {
        // Robot is at the center of the field, facing downfield (0 degrees)
        Pose2d start = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));

        // The mechanism is mounted 0.5 meters directly BEHIND the robot's center 
        // (offsetAngle = 180 degrees relative to robot front)
        // AND its "zero" position faces backwards 
        // (motorZero = 180 degrees relative to robot front)
        Pose2d result = GeometryUtil.getOffsetPose(
            start, 
            0.5, 
            Rotation2d.fromDegrees(180), 
            Rotation2d.fromDegrees(180)
        );

        // Expected Math:
        // Translation: 0.5m at 180 deg -> X = -0.5, Y = 0.0
        // Rotation: Start (0) + MotorZero (180) = 180 degrees
        Pose2d expected = new Pose2d(-0.5, 0.0, Rotation2d.fromDegrees(180));
        assertPoseEquals(expected, result, "Robot at 0 deg, mechanism mounted behind with 180 deg zero");
    }

    @Test
    public void testMotorZeroOffset180_RobotRotated() {
        // Robot is at (2.0, 2.0) but facing LEFT on the field (90 degrees)
        Pose2d start = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(90));

        // Mechanism is mounted 1.0 meter to the robot's RIGHT (-90 or 270 degrees relative to robot)
        // The motor's zero position faces exactly BACKWARDS relative to the robot (180 degrees)
        Pose2d result = GeometryUtil.getOffsetPose(
            start, 
            1.0, 
            Rotation2d.fromDegrees(-90), 
            Rotation2d.fromDegrees(180)
        );

        // Expected Math:
        // 1. Translation: Robot faces North (90). Robot's "Right" (-90 relative) is Field East (0 deg).
        //    So it translates +1.0 along the Field X axis. -> New Pos: (3.0, 2.0).
        // 2. Rotation: Robot faces 90. Motor zero adds 180. 
        //    90 + 180 = 270 degrees (or -90 degrees, facing Field South).
        System.out.println("result.x: " + result.getX());
        System.out.println("result.y: " + result.getY());        
        System.out.println("result.angle: " + result.getRotation().getDegrees());                
        Pose2d expected = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(270));
        assertPoseEquals(expected, result, "Robot at 90 deg, mechanism on right with 180 deg zero");
    }

}