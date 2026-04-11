package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import frc.robot.utilities.LimelightHelpers;

public class EqualWeightSequentialUpdateStrategy  implements IVisionOdometryUpdater {
    public void updateRobotPoseFromVision(
        CommandSwerveDrivetrain drivetrain, 
        LimelightHelpers.PoseEstimate rightLL,
        LimelightHelpers.PoseEstimate leftLL) {
    
    
        if(isValidVisionMeasurement(rightLL)){
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.5,0.5, 999999999));
            drivetrain.addVisionMeasurement(
                rightLL.pose,
                rightLL.timestampSeconds
            ); 
        }

        if(isValidVisionMeasurement(leftLL)){
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.5,0.5, 999999999));
            drivetrain.addVisionMeasurement(
                leftLL.pose,
                leftLL.timestampSeconds
            ); 
        } 
    }

  private boolean isValidVisionMeasurement(LimelightHelpers.PoseEstimate mt2) {
    return mt2 != null && mt2.tagCount > 0;
  }

}
