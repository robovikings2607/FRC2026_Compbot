package frc.robot.subsystems;

import frc.robot.utilities.LimelightHelpers;

public interface IVisionOdometryUpdater {
    void updateRobotPoseFromVision(
        CommandSwerveDrivetrain drivetrain, 
        LimelightHelpers.PoseEstimate rightLL,
        LimelightHelpers.PoseEstimate leftLL);
}
