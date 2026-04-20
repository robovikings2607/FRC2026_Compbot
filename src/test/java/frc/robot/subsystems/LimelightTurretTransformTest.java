package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class LimelightTurretTransformTest {

    private static final double DELTA = 1e-6;

    private static final double TURRET_CENTER_X = 0.1;
    private static final double TURRET_CENTER_Y = -0.05;
    private static final double TURRET_RADIUS   = 0.3;

    // Forward direction: given a known robot pose and turret angle, produce the
    // camera field pose that the Limelight would report as mt.pose.
    private Pose2d toCameraFieldPose(Pose2d robotPose, Rotation2d turretAngle) {
        Translation2d cameraInRobot = new Translation2d(TURRET_CENTER_X, TURRET_CENTER_Y)
                .plus(new Translation2d(TURRET_RADIUS, 0).rotateBy(turretAngle));
        return robotPose.transformBy(new Transform2d(cameraInRobot, turretAngle));
    }

    private void assertPoseEquals(Pose2d expected, Pose2d actual) {
        assertEquals(expected.getX(), actual.getX(), DELTA, "X");
        assertEquals(expected.getY(), actual.getY(), DELTA, "Y");
        assertEquals(expected.getRotation().getDegrees(), actual.getRotation().getDegrees(), DELTA, "heading");
    }

    @Test
    public void testHelpers() {
        // Robot is at (0,0) facing 30 degrees.
        double robotX = 0;
        double robotY = 0;
        Rotation2d robotHeading = Rotation2d.fromDegrees(30);

        // Turret angle is also 30 degrees.
        Rotation2d turretAngle = Rotation2d.fromDegrees(30);

        // Find where the camera is inside the robot:
        double camOffsetX = TURRET_CENTER_X + TURRET_RADIUS * turretAngle.getCos();
        double camOffsetY = TURRET_CENTER_Y + TURRET_RADIUS * turretAngle.getSin();

        // Rotate the above camera-in-robot coordinates by the robot rotation.
        // Also add robotX/Y here - it's zero above, but will make this correct if you adjust.
        // This means the camera will say we're at these coordinates in the field:
        double cameraPoseX = robotX + camOffsetX * robotHeading.getCos() - camOffsetY * robotHeading.getSin();
        double cameraPoseY = robotY + camOffsetX * robotHeading.getSin() + camOffsetY * robotHeading.getCos();
        Rotation2d cameraPoseAngle = Rotation2d.fromDegrees(robotHeading.getDegrees() + turretAngle.getDegrees());

        // Package it into a Pose2d object:
        Pose2d cameraReportedPose = new Pose2d(cameraPoseX, cameraPoseY, cameraPoseAngle);
        Pose2d expectedRobotPose = new Pose2d(robotX, robotY, robotHeading);

        // Give the conversion function the camera pose in the field, make sure we get the robot pose out.
        assertEquals(expectedRobotPose, LimelightSubsystem.cameraToRobotPose(cameraReportedPose, turretAngle, TURRET_CENTER_X, TURRET_CENTER_Y, TURRET_RADIUS));

        // Verify out test function lets us go the other way: field -> camera reported pose
        assertEquals(cameraReportedPose, toCameraFieldPose(expectedRobotPose, turretAngle));
    }

    @Test
    public void turretForward_robotAtOrigin() {
        Pose2d robot = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Rotation2d turret = Rotation2d.fromDegrees(0);
        assertEquals(robot, LimelightSubsystem.cameraToRobotPose(toCameraFieldPose(robot, turret), turret, TURRET_CENTER_X, TURRET_CENTER_Y, TURRET_RADIUS));
    }

    @Test
    public void turretLeft_robotAtOrigin() {
        // Camera swung 90° left — camera sits to the left of robot center in field frame
        Pose2d robot = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Rotation2d turret = Rotation2d.fromDegrees(90);
        assertEquals(robot, LimelightSubsystem.cameraToRobotPose(toCameraFieldPose(robot, turret), turret, TURRET_CENTER_X, TURRET_CENTER_Y, TURRET_RADIUS));
    }

    @Test
    public void turretForward_robotRotated45() {
        // Robot facing 45°, turret pointing forward relative to robot
        Pose2d robot = new Pose2d(3, 4, Rotation2d.fromDegrees(45));
        Rotation2d turret = Rotation2d.fromDegrees(0);
        assertEquals(robot, LimelightSubsystem.cameraToRobotPose(toCameraFieldPose(robot, turret), turret, TURRET_CENTER_X, TURRET_CENTER_Y, TURRET_RADIUS));
    }

    @Test
    public void turretAt45_robotRotated() {
        Pose2d robot = new Pose2d(2, 3, Rotation2d.fromDegrees(30));
        Rotation2d turret = Rotation2d.fromDegrees(45);
        assertEquals(robot, LimelightSubsystem.cameraToRobotPose(toCameraFieldPose(robot, turret), turret, TURRET_CENTER_X, TURRET_CENTER_Y, TURRET_RADIUS));
    }

    @Test
    public void turretBehind_robotAtNonTrivialPose() {
        // Turret pointing 180° (behind robot)
        Pose2d robot = new Pose2d(5, 7, Rotation2d.fromDegrees(-30));
        Rotation2d turret = Rotation2d.fromDegrees(180);
        assertEquals(robot, LimelightSubsystem.cameraToRobotPose(toCameraFieldPose(robot, turret), turret, TURRET_CENTER_X, TURRET_CENTER_Y, TURRET_RADIUS));
    }
}
