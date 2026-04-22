// =============================================================================
// HOW THIS WORKS
// =============================================================================
//
// We have two cameras. Both estimate where the robot is on the field by looking
// at AprilTags (the black-and-white targets mounted around the field).
//
// FRONT CAMERA — bolted to the robot, never moves.
//   Uses MegaTag2 (MT2), which combines what it sees with the gyro heading to
//   get a more stable position estimate. We send it the robot's current heading
//   every loop so it can do that math. Works well with any tags in view.
//
// TURRET CAMERA — mounted on the turret, rotates with it.
//   Only looks at hub tags (IDs 2 and 3). Uses MT1, which doesn't need a
//   heading — it solves for the full camera position from the tags alone.
//
//   We can't use MT2 here because MT2 has two assumptions that break on a
//   rotating camera: (1) it needs to know the camera's heading, which would
//   require sending gyro + turret angle every loop, and (2) it assumes the
//   camera is at a fixed spot on the robot, but our camera orbits the turret
//   center so its position in robot space changes every time the turret moves.
//   MT1 makes neither assumption — it figures out where the camera is from
//   scratch each frame using only what it sees.
//
//   Because the camera spins, we can't just ask "where is the robot?" directly.
//   Instead, we tell the Limelight the camera has no offset from the robot center,
//   so it gives us back the camera's position on the field. Then we do the math:
//
//     camera position on field
//     - camera position on robot  (TURRET_TO_CAM offset, rotating with the turret)
//     = robot position on field
//
//   The turret moves during the ~30ms it takes the camera to capture and process
//   an image, so we can't just use the current turret angle. Instead we log the
//   turret angle every loop into a timestamped history buffer, then look up the
//   angle at the exact moment the image was taken.
//
//   We skip any reading where the turret was spinning fast, since the angle
//   lookup is less reliable and the Limelight solve gets noisier.
//
// =============================================================================
// TUNING TODO LIST
// =============================================================================
//
// Geometry (measure from CAD or on the physical robot):
//   TURRET_CENTER_X / TURRET_CENTER_Y  — where the turret spins from, relative to robot center
//   TURRET_TO_CAM                      — Translation2d from turret pivot to camera lens (at turret angle=0)
//   CAM_PITCH_DEG                      — how far the camera tilts up/down on its mount
//   CAM_ROLL_DEG                       — any sideways tilt on the mount
//   CAM_UP_M                           — camera height above the floor
//   Camera is assumed to be 0 deg relative to the turret
//
// Verify sign conventions:
//   Rotate the turret CCW (left) — confirm getTurretAngleDegrees() increases
//       If it decreases, negate the value in getTurretAngleDegrees()
//
// Filtering thresholds
//   kAmbiguityThreshold     — raise if too many readings are rejected, lower for less noise
//   kMinTagArea             — raise if readings are noisy at long range
//   kMaxTurretRateDegPerSec — lower if poses glitch during fast slews; raise if too restrictive
//   kMaxHeadingErrorDeg     — how far the MT1 heading can disagree with the gyro before rejecting
//
// =============================================================================

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.RobotLogger;

public class LimelightSubsystem extends SubsystemBase {

  // -------------------------------------------------------------------------
  // Camera names
  // -------------------------------------------------------------------------
  private static final String FRONT_NAME  = "limelight-front";
  private static final String TURRET_NAME = "limelight-back";

  // Only look at hub tags with the turret camera — ignore everything else
  private static final int[] HUB_TAG_IDS = {2, 3, 4, 5, 8, 9 , 10, 11, 18, 19, 20, 21, 24, 25, 26, 27};
  private static final int[] FRONT_CAM_BLACKLIST = {};

  // -------------------------------------------------------------------------
  // Turret camera geometry (TODO: fill from CAD measurements)
  // -------------------------------------------------------------------------

  // Where the turret spins from, measured from the robot center (meters)
  private static final double TURRET_CENTER_X = Units.inchesToMeters(-6.75);
  private static final double TURRET_CENTER_Y = Units.inchesToMeters(-5.75);

  // Camera position relative to turret pivot, in the turret's own frame (X+ = turret forward, Y+ = turret left).
  // At turret angle=0 this frame is aligned with the robot frame.
  private static final Translation2d TURRET_TO_CAM = new Translation2d(Units.inchesToMeters(0.25), Units.inchesToMeters(4.625));

  // Which way the camera points when the turret encoder reads 0, relative to robot forward
  // (degrees, positive = counterclockwise)
  private static final double CAM_YAW_OFFSET_DEG = 0.0;

  // Camera tilt — these don't change as the turret rotates (TODO: measure)
  private static final double CAM_PITCH_DEG = 0.0;
  private static final double CAM_ROLL_DEG  = 0.0;

  // How high the camera is off the ground (meters)
  private static final double CAM_UP_M = 0.62865;

  // -------------------------------------------------------------------------
  // Filtering constants
  // -------------------------------------------------------------------------
  private static final double kAmbiguityThreshold = 0.3;
  private static final double kMinTagArea         = 0.1;
  private static final double kMaxTagArea         = 5.5;
  private static final double kMinPoseNorm           = 0.5;
  private static final double kMaxHeadingErrorDeg    = 10.0;

  // Skip turret camera readings when the turret is spinning too fast — at high
  // speed the angle we look up may be wrong, and the LL pose solve gets noisy.
  // This also catches the turret snapping back when it hits a soft limit.
  private static final double kMaxTurretRateDegPerSec = 60.0;

  // Static position uncertainty (meters) for each camera.
  // We also tell the filter to never correct gyro heading from vision (1e9).
  private static final double kFrontStd  = 0.2;
  private static final double kTurretStd = 0.5;
  private static final double kLargeVariance = 1e9;

  // -------------------------------------------------------------------------
  // Turret angle + speed snapshot, stored every loop and looked up by timestamp
  // -------------------------------------------------------------------------
  private static class TurretState {
    // storing the raw double here instead of Rotation2d, since turret uses 0-360 - which will interpolate incorrectly with a [-180, 180] bound Rotation2d
    final double angleDeg;
    final double rateDegsPerSec;

    TurretState(double angleDeg, double rateDegsPerSec) {
      this.angleDeg       = angleDeg;
      this.rateDegsPerSec = rateDegsPerSec;
    }

    static TurretState interpolate(TurretState a, TurretState b, double t) {
      return new TurretState(
          MathUtil.interpolate(a.angleDeg, b.angleDeg, t),
          MathUtil.interpolate(a.rateDegsPerSec, b.rateDegsPerSec, t));
    }
  }

  // -------------------------------------------------------------------------
  // State
  // -------------------------------------------------------------------------
  private final RobotContainer robot;

  // Stores the last 0.5 s of turret snapshots so we can look up where the
  // turret was pointing at the exact moment the camera captured an image
  private final TimeInterpolatableBuffer<TurretState> turretBuffer =
      TimeInterpolatableBuffer.createBuffer(TurretState::interpolate, 0.5);

  private AprilTagFieldLayout tagLayout;
  private double lastSubmittedTimestamp = 0.0;

  // -------------------------------------------------------------------------
  // Inner type
  // -------------------------------------------------------------------------
  private static class CameraEstimate {
    final Pose2d pose;
    final double timestampSeconds;
    final Matrix<N3, N1> stdDevs;
    final int tagCount;

    CameraEstimate(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs, int tagCount) {
      this.pose             = pose;
      this.timestampSeconds = timestampSeconds;
      this.stdDevs          = stdDevs;
      this.tagCount         = tagCount;
    }
  }

  // -------------------------------------------------------------------------
  // Constructor
  // -------------------------------------------------------------------------
  public LimelightSubsystem(RobotContainer robot) {
    this.robot = robot;

    // Front camera: bolted to the robot, uses MegaTag2 (requires us to send the robot's heading)
    LimelightHelpers.setPipelineIndex(FRONT_NAME, 0);
    LimelightHelpers.SetIMUMode(FRONT_NAME, 1);

    // Turret camera: only look at hub tags, use MT1 (no heading needed)
    LimelightHelpers.setPipelineIndex(TURRET_NAME, 0);
    LimelightHelpers.SetIMUMode(TURRET_NAME, 0);
    LimelightHelpers.SetFiducialIDFiltersOverride(TURRET_NAME, HUB_TAG_IDS);

    // Tell the LL the camera has no horizontal offset from the robot center.
    // This makes mt.pose equal to the camera's position on the field,
    // so we can do the offset math ourselves using the actual turret angle.
    // Yaw is 0 here — we add turret rotation ourselves each loop.
    // Pitch and roll are the fixed tilt of the camera on its mount.
    LimelightHelpers.setCameraPose_RobotSpace(TURRET_NAME, 0, 0, CAM_UP_M, CAM_ROLL_DEG, CAM_PITCH_DEG, 0);

    try {
      java.io.File fieldJsonFile = new java.io.File(
          edu.wpi.first.wpilibj.Filesystem.getDeployDirectory(), "2026-rebuilt-welded.json");
      tagLayout = new AprilTagFieldLayout(fieldJsonFile.toPath());
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  // -------------------------------------------------------------------------
  // Periodic
  // -------------------------------------------------------------------------
  @Override
  public void periodic() {
    double yaw      = robot.drivetrain.getState().Pose.getRotation().getDegrees();
    double fpgaTime = Timer.getFPGATimestamp();

    turretBuffer.addSample(fpgaTime, new TurretState(
        robot.turret.getTurretAngleDegrees(),
        robot.turret.getTurretRateDegPerSec()));

    // --- Front camera (fixed, MT2) ---
    LimelightHelpers.SetRobotOrientation(FRONT_NAME, yaw, 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate frontRaw =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(FRONT_NAME);
    Optional<CameraEstimate> frontEst = processFrontCamera(frontRaw);

    Pose2d frontVizPose = frontEst.isPresent() ? frontRaw.pose : Constants.EMPTY_POSE;
    updateFieldVisualization(frontVizPose, getTagPoses(frontRaw), FRONT_NAME);

    // --- Turret camera (MT1) ---
    LimelightHelpers.PoseEstimate turretRaw =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(TURRET_NAME);
    Optional<CameraEstimate> turretEst = processTurretCamera(turretRaw);

    Pose2d turretVizPose = turretEst.isPresent() ? turretEst.get().pose : Constants.EMPTY_POSE;
    updateFieldVisualization(turretVizPose, getTagPoses(turretRaw), TURRET_NAME);

    // Prefer turret camera (locked on hub) over front camera
    Optional<CameraEstimate> toSubmit = turretEst.isPresent() ? turretEst : frontEst;

    toSubmit.ifPresent(est -> {
      if (est.timestampSeconds > lastSubmittedTimestamp) {
        robot.drivetrain.addVisionMeasurement(est.pose, est.timestampSeconds, est.stdDevs);
        lastSubmittedTimestamp = est.timestampSeconds;
        RobotLogger.logStruct("Limelight/Robot/pose", Pose2d.struct, est.pose);
        RobotLogger.logDouble("Limelight/Robot/timestamp", est.timestampSeconds);
      }
    });
  }

  // -------------------------------------------------------------------------
  // Front camera (fixed mount, MT2)
  // -------------------------------------------------------------------------
  private Optional<CameraEstimate> processFrontCamera(LimelightHelpers.PoseEstimate mt) {
    if (mt == null || mt.tagCount == 0) return Optional.empty();
    if (mt.timestampSeconds <= lastSubmittedTimestamp) return Optional.empty();
    
    if (mt.pose.getTranslation().getNorm() < kMinPoseNorm) return Optional.empty();
    if(hasBlackListedTags(mt, FRONT_CAM_BLACKLIST)) return Optional.empty();

    Matrix<N3, N1> stdDevs = VecBuilder.fill(kFrontStd, kFrontStd, kLargeVariance);

    return Optional.of(new CameraEstimate(mt.pose, mt.timestampSeconds, stdDevs, mt.tagCount));
  }

  // -------------------------------------------------------------------------
  // Turret camera (MT1)
  // -------------------------------------------------------------------------
  private Optional<CameraEstimate> processTurretCamera(LimelightHelpers.PoseEstimate mt) {      RobotLogger.logBoolean("Limelight/" + TURRET_NAME + "/noTags?", mt == null || mt.tagCount == 0);
    RobotLogger.logBoolean("Limelight/" + TURRET_NAME + "/noTags?", mt == null || mt.tagCount == 0);
    if (mt == null || mt.tagCount == 0) {
      return Optional.empty();
    }

    RobotLogger.logBoolean("Limelight/" + TURRET_NAME + "/badTimestamp", mt.timestampSeconds <= lastSubmittedTimestamp);
    if (mt.timestampSeconds <= lastSubmittedTimestamp){ 
      return Optional.empty();}

    if (mt.tagCount == 1 && mt.rawFiducials != null) {
      for (LimelightHelpers.RawFiducial f : mt.rawFiducials) {
        RobotLogger.logBoolean("Limelight/" + TURRET_NAME + "/badAmbiquity", f.ambiguity > kAmbiguityThreshold);
        RobotLogger.logDouble("Limelight/" + TURRET_NAME + "/Ambiquity", f.ambiguity);
        if (f.ambiguity > kAmbiguityThreshold) return Optional.empty();
      }
    }

    RobotLogger.logBoolean("Limelight/" + TURRET_NAME + "/badArea", mt.avgTagArea < kMinTagArea || mt.avgTagArea > kMaxTagArea);
    if (mt.avgTagArea < kMinTagArea || mt.avgTagArea > kMaxTagArea) return Optional.empty();

    RobotLogger.logBoolean("Limelight/" + TURRET_NAME + "/badNorm", mt.pose.getTranslation().getNorm() < kMinPoseNorm);
    if (mt.pose.getTranslation().getNorm() < kMinPoseNorm) return Optional.empty();

    // Find out where the turret was pointing when this image was taken.
    // If the image is too old to be in our history, skip it.
    Optional<TurretState> stateOpt = turretBuffer.getSample(mt.timestampSeconds);
    RobotLogger.logBoolean("Limelight/" + TURRET_NAME + "/badSample", stateOpt.isEmpty());
    if (stateOpt.isEmpty()) return Optional.empty();
    TurretState state = stateOpt.get();

    // Skip this reading if the turret was spinning fast when the photo was taken.
      RobotLogger.logBoolean("Limelight/"+TURRET_NAME +"/badRate?", Math.abs(state.rateDegsPerSec) > kMaxTurretRateDegPerSec);
    if (Math.abs(state.rateDegsPerSec) > kMaxTurretRateDegPerSec) return Optional.empty();

    // Because we told the LL the camera is at the robot center (no offset),
    // mt.pose is where the camera is on the field. From there, we subtract
    // the camera's position on the robot (which rotates with the turret)
    // to get where the robot center actually is.
    Rotation2d turretAngle = Rotation2d.fromDegrees(state.angleDeg + CAM_YAW_OFFSET_DEG).unaryMinus();
    Pose2d robotPose = cameraToRobotPose(mt.pose, turretAngle, TURRET_CENTER_X, TURRET_CENTER_Y, TURRET_TO_CAM);

    // If the pose is right at (0,0), it's probably garbage from the LL.
    if (robotPose.getTranslation().getNorm() < kMinPoseNorm) return Optional.empty();

    // MT1 also estimates which way the robot is facing. If that disagrees
    // with the gyro by more than kMaxHeadingErrorDeg, something went wrong.
    Rotation2d gyroYaw = robot.drivetrain.getState().Pose.getRotation();
    double headingError = Math.abs(robotPose.getRotation().minus(gyroYaw).getDegrees());
    RobotLogger.logBoolean("Limelight/"+TURRET_NAME +"/badHeading?", headingError > kMaxHeadingErrorDeg);
    if (headingError > kMaxHeadingErrorDeg) return Optional.empty();

    Matrix<N3, N1> stdDevs = VecBuilder.fill(kTurretStd, kTurretStd, kLargeVariance);

    return Optional.of(new CameraEstimate(robotPose, mt.timestampSeconds, stdDevs, mt.tagCount));
  }

  // -------------------------------------------------------------------------
  // Helpers
  // -------------------------------------------------------------------------

  public static Pose2d cameraToRobotPose(
      Pose2d cameraPose, Rotation2d turretAngle,
      double turretCenterX, double turretCenterY, Translation2d turretToCam) {

    // turretAngle is used twice here:
    // once to rotate the turret->camera offset vector into the robot frame
    // and once as the camera's heading in the robot frame
    Translation2d cameraTranslation = new Translation2d(turretCenterX, turretCenterY)  // start at turret pivot in robot frame
        .plus(turretToCam.rotateBy(turretAngle));                                      // add cam offset rotated to current turret angle

    Transform2d robotToCamera = new Transform2d(cameraTranslation, turretAngle);        // camera pose in robot frame (position + heading)
    return cameraPose.transformBy(robotToCamera.inverse());                             // camera field pose -> robot field pose
  }
private void updateFieldVisualization(Pose2d robotPose, List<Pose2d> tagPoses, String name) {
    RobotLogger.logStruct("Limelight/" + name + "/RobotPose", Pose2d.struct, robotPose);
    RobotLogger.logStructArray("Limelight/" + name + "/AprilTagPoses",
        Pose2d.struct, tagPoses.toArray(Pose2d[]::new));
  }

  public List<Pose2d> getTagPoses(LimelightHelpers.PoseEstimate mt) {
    ArrayList<Pose2d> tagPoses = new ArrayList<>();
    if (tagLayout != null && mt != null && mt.rawFiducials != null) {
      for (LimelightHelpers.RawFiducial tag : mt.rawFiducials) {
        var pose3d = tagLayout.getTagPose(tag.id);
        if (pose3d.isPresent()) {
          tagPoses.add(pose3d.get().toPose2d());
        }
      }
    }
    return tagPoses;
  }

  private boolean hasBlackListedTags(LimelightHelpers.PoseEstimate cameraPose, int[] blackList) {
    for (int i = 0; i < cameraPose.rawFiducials.length; i++) {
      for (int j = 0; j < blackList.length; j++) {
        if (cameraPose.rawFiducials[i].id == blackList[j]) { return true; }
      }
    }
    return false;
  }
}
