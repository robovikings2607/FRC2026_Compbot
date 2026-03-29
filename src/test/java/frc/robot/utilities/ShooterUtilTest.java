package frc.robot.utilities;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utilities.GeometryUtil;

public class ShooterUtilTest {

    @Test
    public void testRobot350TargetMinus150Degrees1() {
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

        double turretAngle = ShooterUtils.testTurretAngle1(robotPose, targetPose.getTranslation());

        System.out.println("Turret Angle 1: " + turretAngle);

    }

    @Test
    public void testRobot350TargetMinus150Degrees2() {
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

        double turretAngle = ShooterUtils.testTurretAngle2(robotPose, targetPose.getTranslation());

        System.out.println("Turret Angle 2: " + turretAngle);

    }
}