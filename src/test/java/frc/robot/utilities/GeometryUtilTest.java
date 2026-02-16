package frc.robot.utilities;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GeometryUtilTest {

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

}