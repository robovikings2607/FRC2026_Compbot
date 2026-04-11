package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.RobotLogger;

public class FusedPoseVisionUpdateStrategy implements IVisionOdometryUpdater{

    private double lastSubmittedTimestamp = 0.0;

    /** Reject single-tag estimates with pose ambiguity above this threshold. */
    private static final double kAmbiguityThreshold = 1e9; //0.3

    /**
     * Minimum average tag area (% of image) required for single-tag estimates.
     * Multi-tag estimates use the lower kMinTagAreaMulti threshold.
     */
    private static final double kMinTagAreaSingle = 0.2;
    private static final double kMinTagAreaMulti  = 0.05;

    private static final double kMaxTagArea = 4.9;

    /**
     * Reject any pose whose translation norm is below this value.
     * Catches bad solves that report a position near the field origin.
     */
    private static final double kMinPoseNorm = 0.5; // meters

    /**
     * Base XY standard deviation for multi-tag and single-tag estimates (meters).
     * Scaled up linearly with average tag distance.
     */
    private static final double kMultiTagBaseStd  = 0.2;
    private static final double kSingleTagBaseStd = 0.4;

    /** Per-meter distance scaling factor applied to base stddev. */
    private static final double kDistStdScale = 0.05;

    /** Large variance used for the rotation component — we trust the gyro, not vision heading. */
    private static final double kLargeVariance = 1e9;

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


    public void updateRobotPoseFromVision(
        CommandSwerveDrivetrain drivetrain, 
        LimelightHelpers.PoseEstimate rightLL,
        LimelightHelpers.PoseEstimate leftLL) {

        Optional<CameraEstimate> leftEst  = processCamera(leftLL,  LimelightSubsystem.LEFT_LIMELIGHT_NAME);
        Optional<CameraEstimate> rightEst = processCamera(rightLL, LimelightSubsystem.RIGHT_LIMELIGHT_NAME);

        // Fuse both cameras before submitting a single update to the pose estimator.
        // This avoids two correlated updates hitting the Kalman filter independently.
        Optional<CameraEstimate> toSubmit;
        if (leftEst.isPresent() && rightEst.isPresent()) {
            toSubmit = Optional.of(fuseEstimates(leftEst.get(), rightEst.get()));
        } else if (leftEst.isPresent()) {
            toSubmit = leftEst;
        } else {
            toSubmit = rightEst;
        }   
        
        toSubmit.ifPresent(est -> {
            if (est.timestampSeconds > lastSubmittedTimestamp) {
                drivetrain.addVisionMeasurement(est.pose, est.timestampSeconds, est.stdDevs);
                lastSubmittedTimestamp = est.timestampSeconds;
                SmartDashboard.putNumber("Robot/fusedtimestamped", est.timestampSeconds);
                RobotLogger.logStruct("Limelight/" + "Robot/robotPose", Pose2d.struct, est.pose);   
            }             
        });
        
    }

  // -------------------------------------------------------------------------
  // Per-camera filtering
  // -------------------------------------------------------------------------

  /**
   * Applies 254-inspired filtering to a raw MegaTag2 estimate.
   *
   * <p>Checks applied:
   * <ol>
   *   <li>Null / zero-tag guard</li>
   *   <li>Single-tag: ambiguity threshold</li>
   *   <li>Single-tag: minimum tag area</li>
   *   <li>Multi-tag: minimum tag area</li>
   *   <li>Pose norm check (rejects solves near the field origin)</li>
   *   <li>Timestamp deduplication</li>
   * </ol>
   *
   * <p>Standard deviations are scaled with average tag distance so that
   * farther (less reliable) observations receive proportionally less weight
   * in the Kalman filter.
   */
  private Optional<CameraEstimate> processCamera(LimelightHelpers.PoseEstimate mt, String name) {
    if (mt == null || mt.tagCount == 0) {
      SmartDashboard.putBoolean("Limelight/" + name + "/no Tags", true);
      return Optional.empty();
    }
    SmartDashboard.putBoolean("Limelight/" + name + "/no Tags", false);

    if (mt.timestampSeconds <= lastSubmittedTimestamp) {
      SmartDashboard.putBoolean("Limelight/" + name + "/bad Timestamp", true);
      return Optional.empty();
    }
    SmartDashboard.putBoolean("Limelight/" + name + "/bad Timestamp", false);

    if (mt.tagCount < 2) {
      // Single-tag extra checks
      if (mt.rawFiducials != null) {
        for (LimelightHelpers.RawFiducial f : mt.rawFiducials) {
          if (f.ambiguity > kAmbiguityThreshold) {
            SmartDashboard.putBoolean("Limelight/" + name + "/bad Ambuguity", true);
            return Optional.empty();
          }
        }
      }
      SmartDashboard.putBoolean("Limelight/" + name + "/bad Ambuguity", false);

      if (mt.avgTagArea < kMinTagAreaSingle || mt.avgTagArea > kMaxTagArea) {
        SmartDashboard.putBoolean("Limelight/" + name + "/bad Area", true);
        return Optional.empty();
      }
      SmartDashboard.putBoolean("Limelight/" + name + "/bad Area", false);
    } 
    else {
      if (mt.avgTagArea < kMinTagAreaMulti) {
        SmartDashboard.putBoolean("Limelight/" + name + "/bad MinTagAreaMulti", true);
        return Optional.empty();
      }
      SmartDashboard.putBoolean("Limelight/" + name + "/bad MinTagAreaMulti", false);
    }

    // Reject poses suspiciously close to the field origin (likely a bad solve)
    if (mt.pose.getTranslation().getNorm() < kMinPoseNorm) {
      SmartDashboard.putBoolean("Limelight/" + name + "/bad PoseNorm", true);
      return Optional.empty();
    }
    SmartDashboard.putBoolean("Limelight/" + name + "/bad PoseNorm", false);

    // Quality-scaled standard deviations.
    // quality = 1.0 for multi-tag; 1.0 - ambiguity for single-tag (per 254's metric).
    // stddev scales with 1/quality so higher ambiguity → less trust.
    // An additional distance term accounts for tag size / measurement noise at range.
    double quality = (mt.tagCount >= 2) ? 1.0
        : 1.0 - mt.rawFiducials[0].ambiguity;
    double baseStd = (mt.tagCount >= 2) ? kMultiTagBaseStd : kSingleTagBaseStd;
    double xyStd   = baseStd * (1.0 / quality) * (1.0 + kDistStdScale * mt.avgTagDist);
    Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStd, xyStd, kLargeVariance);

    return Optional.of(new CameraEstimate(mt.pose, mt.timestampSeconds, stdDevs, mt.tagCount));
  }

  // -------------------------------------------------------------------------
  // Inverse-variance weighted fusion of two camera estimates
  // -------------------------------------------------------------------------

  /**
   * Fuses two per-camera estimates into one using inverse-variance weighting,
   * matching 254's approach in VisionSubsystem.fuseEstimates().
   *
   * <p>Because both cameras share the same gyro via MegaTag2, we keep the
   * rotation from the more recent estimate and only fuse translation.
   */
  private CameraEstimate fuseEstimates(CameraEstimate a, CameraEstimate b) {
    // Work in variance space
    double varAx = sq(a.stdDevs.get(0, 0));
    double varAy = sq(a.stdDevs.get(1, 0));
    double varBx = sq(b.stdDevs.get(0, 0));
    double varBy = sq(b.stdDevs.get(1, 0));

    double wAx = 1.0 / varAx, wBx = 1.0 / varBx;
    double wAy = 1.0 / varAy, wBy = 1.0 / varBy;

    Pose2d fusedPose = new Pose2d(
        new Translation2d(
            (a.pose.getX() * wAx + b.pose.getX() * wBx) / (wAx + wBx),
            (a.pose.getY() * wAy + b.pose.getY() * wBy) / (wAy + wBy)),
        // Both rotations are gyro-derived via MT2; use the newer one
        (a.timestampSeconds >= b.timestampSeconds)
            ? a.pose.getRotation()
            : b.pose.getRotation());

    Matrix<N3, N1> fusedStdDevs = VecBuilder.fill(
        Math.sqrt(1.0 / (wAx + wBx)),
        Math.sqrt(1.0 / (wAy + wBy)),
        kLargeVariance);

    double fusedTimestamp = Math.max(a.timestampSeconds, b.timestampSeconds);
    int fusedTagCount     = a.tagCount + b.tagCount;

    return new CameraEstimate(fusedPose, fusedTimestamp, fusedStdDevs, fusedTagCount);
  }

  private static double sq(double x) { return x * x; }

    
}
