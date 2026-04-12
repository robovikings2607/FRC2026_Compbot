// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utilities.LimelightHelpers;

/**
 * Computes the 3D camera-to-robot transform for a camera whose physical
 * mounting offset is unknown or untrusted.
 *
 * ── COORDINATE SYSTEMS ─────────────────────────────────────────────────────
 *   WPILib robot frame  :  X+ forward, Y+ LEFT,  Z+ up  (NWU / right-handed)
 *   Limelight Robot Space: X+ forward, Y+ RIGHT, Z+ up
 *
 *   The output of this command is reported in Limelight Robot Space
 *   (forward / side / up / roll / pitch / yaw) as expected by the
 *   Limelight dashboard and setCameraPose_RobotSpace().
 * ───────────────────────────────────────────────────────────────────────────
 *
 * ── APPROACH ────────────────────────────────────────────────────────────────
 *   Uses the MegaTag2 (botpose_orb_wpiblue) pose from the calibration camera.
 *
 *   MegaTag2 reports the robot pose using the currently-configured camera
 *   offset. To recover the camera's true field pose we compose:
 *
 *     cameraPose_field = robotPose_fromMT2.transformBy(configuredOffset_WPI)
 *
 *   Then the corrected robot-to-camera transform is:
 *
 *     corrected = Transform3d(robotPose_groundTruth, cameraPose_field)
 *
 *   where robotPose_groundTruth comes from drivetrain odometry seeded by
 *   a trusted camera. Averaging SAMPLES_REQUIRED of these gives the final
 *   result.
 *
 *   The command also reports the delta between the corrected transform and
 *   the currently-configured offset — add the deltas to your existing config
 *   to correct it without re-entering all values from scratch.
 *
 * ── PROCEDURE ───────────────────────────────────────────────────────────────
 *   1. Point the TRUSTED camera at AprilTags; let drivetrain pose stabilise.
 *   2. Rotate so the CALIBRATION camera sees those same tags (or other tags
 *      visible from the same known robot pose).
 *   3. Run this command. It collects samples until SAMPLES_REQUIRED is met
 *      then prints the result to SmartDashboard and the console.
 *   4. Either enter the new absolute values into Limelight dashboard
 *      → Camera Pose in Robot Space, OR apply the printed deltas to the
 *      existing config.
 *
 * ── NOTES ───────────────────────────────────────────────────────────────────
 *   • The calibration camera's vision contribution to the drivetrain estimator
 *     is disabled while this command runs so its (possibly wrong) offset does
 *     not contaminate the ground-truth pose.
 *   • MegaTag2 fixes yaw via the gyro, so the recovered camera field pose
 *     orientation reflects the gyro heading, not a full optical solve.
 *     Yaw/roll/pitch corrections are most reliable when the camera offset is
 *     close to correct to begin with; use the per-tag command for large
 *     rotational errors.
 */
public class CameraCalibrationCommand extends Command {

    /** Minimum average tag area (% of image) — filters out distant/noisy MT2 readings. */
    private static final double MIN_TAG_AREA = 0.05;

    /** Number of accepted samples before computing the final result. */
    private static final int SAMPLES_REQUIRED = 100;

    private final RobotContainer robot;
    private final String calibrateCameraName;

    private final List<Transform3d> samples = new ArrayList<>();
    private boolean done = false;

    /**
     * @param robot               the RobotContainer
     * @param calibrateCameraName name of the Limelight to calibrate
     *                            (e.g. "limelight-front")
     */
    public CameraCalibrationCommand(RobotContainer robot, String calibrateCameraName) {
        this.robot = robot;
        this.calibrateCameraName = calibrateCameraName;
    }

    @Override
    public void initialize() {
        samples.clear();
        done = false;
        // Disable the calibration camera from updating the drivetrain estimator.
        // Without this, the camera's bad offset contaminates the very pose we're
        // using as the ground-truth reference.
        robot.limelight.setVisionEnabled(calibrateCameraName, false);

        System.err.println("[CameraCalibration] ==============================");
        SmartDashboard.putString("CameraCalibration/Status", "Collecting...");
        SmartDashboard.putNumber("CameraCalibration/Samples", 0);
    }

    @Override
    public void execute() {
        // Ground truth: drivetrain pose continuously updated by odometry (and
        // previously seeded by the trusted camera).
        Pose3d robotPose_field = new Pose3d(robot.drivetrain.getState().Pose);

        // MegaTag2 pose from the calibration camera.
        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(calibrateCameraName);

        if (mt2 == null || mt2.tagCount == 0 || mt2.avgTagArea < MIN_TAG_AREA) {
            SmartDashboard.putString("CameraCalibration/Status",
                "No tags / bad area (" + samples.size() + " samples)");
            return;
        }

        // Currently-configured camera-to-robot transform (Limelight Robot Space → WPILib).
        // MT2 computed its robot pose using this offset, so we need it to recover the
        // camera's field pose.
        Pose3d configuredCamPose_LL = LimelightHelpers.getCameraPose3d_RobotSpace(calibrateCameraName);
        Transform3d configuredCam_WPI = llRobotSpaceToWpi(configuredCamPose_LL);

        // Recover camera field pose:
        //   MT2 derives robotPose by removing the configured camera offset from its
        //   optical camera-field-pose measurement.  Compose in reverse to get it back.
        Pose3d robotPose_fromCamera = new Pose3d(mt2.pose);
        Pose3d cameraPose_field = robotPose_fromCamera.transformBy(configuredCam_WPI);

        // Corrected robot-to-camera transform using the ground-truth robot pose.
        Transform3d corrected_WPI = new Transform3d(robotPose_field, cameraPose_field);
        samples.add(corrected_WPI);

        SmartDashboard.putNumber("CameraCalibration/Samples", samples.size());
        SmartDashboard.putNumber("CameraCalibration/TagCount", mt2.tagCount);

        if (samples.size() >= SAMPLES_REQUIRED) {
            Transform3d avg_WPI = averageSamples(samples);
            logResult(avg_WPI, configuredCam_WPI);
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        // Always re-enable, whether we finished normally or were cancelled.
        //robot.limelight.setVisionEnabled(calibrateCameraName, true);

        SmartDashboard.putString("CameraCalibration/Status", "Exited");
        if (interrupted && samples.size() > 10) {
            System.out.printf("[CameraCalibration] Interrupted — partial result from %d samples:%n",
                samples.size());
            Pose3d configuredCamPose_LL = LimelightHelpers.getCameraPose3d_RobotSpace(calibrateCameraName);
            logResult(averageSamples(samples), llRobotSpaceToWpi(configuredCamPose_LL));
        }
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    /**
     * Converts a camera pose from Limelight Robot Space (X+ forward, Y+ right, Z+ up)
     * to a WPILib-frame Transform3d (X+ forward, Y+ left, Z+ up).
     *
     * Mirroring Y negates the Y translation component and flips the sense of
     * roll and yaw (rotations about axes that now point the opposite direction).
     */
    private static Transform3d llRobotSpaceToWpi(Pose3d llPose) {
        return new Transform3d(
            new Translation3d(llPose.getX(), -llPose.getY(), llPose.getZ()),
            new Rotation3d(
                -llPose.getRotation().getX(),   // roll: sense flips with Y mirror
                 llPose.getRotation().getY(),    // pitch: unchanged
                -llPose.getRotation().getZ())); // yaw: sense flips with Y mirror
    }

    /**
     * Component-wise average of Transform3d values.
     * Translations are mean-averaged. Quaternions are sign-normalised to the
     * same hemisphere as the first sample, then mean-averaged and re-normalised.
     */
    private static Transform3d averageSamples(List<Transform3d> list) {
        double tx = 0, ty = 0, tz = 0;
        double qw = 0, qx = 0, qy = 0, qz = 0;

        Quaternion ref = list.get(0).getRotation().getQuaternion();

        for (Transform3d t : list) {
            tx += t.getX();
            ty += t.getY();
            tz += t.getZ();

            Quaternion q = t.getRotation().getQuaternion();
            double dot = q.getW()*ref.getW() + q.getX()*ref.getX()
                       + q.getY()*ref.getY() + q.getZ()*ref.getZ();
            double sign = (dot < 0) ? -1.0 : 1.0;
            qw += sign * q.getW();
            qx += sign * q.getX();
            qy += sign * q.getY();
            qz += sign * q.getZ();
        }

        int n = list.size();
        double mag = Math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz);

        return new Transform3d(
            new Translation3d(tx / n, ty / n, tz / n),
            new Rotation3d(new Quaternion(qw / mag, qx / mag, qy / mag, qz / mag))
        );
    }

    /**
     * Logs the corrected camera-to-robot transform (in Limelight Robot Space)
     * and the delta vs. the currently-configured offset.
     *
     * WPILib → Limelight Robot Space conversion:
     *   forward = +X            (unchanged)
     *   side    = -Y            (Y+ left → Y+ right)
     *   up      = +Z            (unchanged)
     *   roll    = -Rx           (sense flips with Y mirror)
     *   pitch   = +Ry           (unchanged)
     *   yaw     = -Rz           (sense flips with Y mirror)
     */
    private void logResult(Transform3d corrected_WPI, Transform3d configured_WPI) {
        // Corrected values in Limelight Robot Space
        double fwd   = corrected_WPI.getX();
        double side  = -corrected_WPI.getY();
        double up    = corrected_WPI.getZ();
        double roll  = -Units.radiansToDegrees(corrected_WPI.getRotation().getX());
        double pitch =  Units.radiansToDegrees(corrected_WPI.getRotation().getY());
        double yaw   = -Units.radiansToDegrees(corrected_WPI.getRotation().getZ());

        // Currently-configured values in Limelight Robot Space
        double cfgFwd   = configured_WPI.getX();
        double cfgSide  = -configured_WPI.getY();
        double cfgUp    = configured_WPI.getZ();
        double cfgRoll  = -Units.radiansToDegrees(configured_WPI.getRotation().getX());
        double cfgPitch =  Units.radiansToDegrees(configured_WPI.getRotation().getY());
        double cfgYaw   = -Units.radiansToDegrees(configured_WPI.getRotation().getZ());

        // Delta = corrected − configured  (add this to your existing config)
        double dFwd   = fwd   - cfgFwd;
        double dSide  = side  - cfgSide;
        double dUp    = up    - cfgUp;
        double dRoll  = MathUtil.inputModulus(roll  - cfgRoll,  -180.0, 180.0);
        double dPitch = MathUtil.inputModulus(pitch - cfgPitch, -180.0, 180.0);
        double dYaw   = MathUtil.inputModulus(yaw   - cfgYaw,   -180.0, 180.0);

        SmartDashboard.putString("CameraCalibration/Status", "Done! (" + samples.size() + " samples)");

        // Absolute corrected values (enter these directly into LL dashboard)
        SmartDashboard.putNumber("CameraCalibration/LL/forward_m", fwd);
        SmartDashboard.putNumber("CameraCalibration/LL/side_m",    side);
        SmartDashboard.putNumber("CameraCalibration/LL/up_m",      up);
        SmartDashboard.putNumber("CameraCalibration/LL/roll_deg",  roll);
        SmartDashboard.putNumber("CameraCalibration/LL/pitch_deg", pitch);
        SmartDashboard.putNumber("CameraCalibration/LL/yaw_deg",   yaw);

        // Delta from current config (add these to the existing values)
        SmartDashboard.putNumber("CameraCalibration/Delta/forward_m", dFwd);
        SmartDashboard.putNumber("CameraCalibration/Delta/side_m",    dSide);
        SmartDashboard.putNumber("CameraCalibration/Delta/up_m",      dUp);
        SmartDashboard.putNumber("CameraCalibration/Delta/roll_deg",  dRoll);
        SmartDashboard.putNumber("CameraCalibration/Delta/pitch_deg", dPitch);
        SmartDashboard.putNumber("CameraCalibration/Delta/yaw_deg",   dYaw);

        System.out.printf("[CameraCalibration] === Result for '%s' (%d samples) ===%n",
                          calibrateCameraName, samples.size());
        System.out.printf("  Corrected Camera Pose in Robot Space (enter into Limelight dashboard):%n");
        System.out.printf("  Forward = %+.4f m%n", fwd);
        System.out.printf("  Side    = %+.4f m  (+ = right of robot center)%n", side);
        System.out.printf("  Up      = %+.4f m%n", up);
        System.out.printf("  Roll    = %+.2f deg%n", roll);
        System.out.printf("  Pitch   = %+.2f deg%n", pitch);
        System.out.printf("  Yaw     = %+.2f deg  (+ = CW from robot forward, toward robot right)%n", yaw);
        System.out.printf("  Delta to apply to existing config (corrected − configured):%n");
        System.out.printf("  ΔForward = %+.4f m%n", dFwd);
        System.out.printf("  ΔSide    = %+.4f m%n", dSide);
        System.out.printf("  ΔUp      = %+.4f m%n", dUp);
        System.out.printf("  ΔRoll    = %+.2f deg%n", dRoll);
        System.out.printf("  ΔPitch   = %+.2f deg%n", dPitch);
        System.out.printf("  ΔYaw     = %+.2f deg%n", dYaw);
    }
}
