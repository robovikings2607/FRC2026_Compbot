package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimelightConstants;
import frc.robot.RobotContainer;
import frc.robot.utilities.LimelightHelpers;

public class StdDevVisionUpdateStrategy implements IVisionOdometryUpdater{
    public void updateRobotPoseFromVision(
        CommandSwerveDrivetrain drivetrain, 
        LimelightHelpers.PoseEstimate rightLL,
        LimelightHelpers.PoseEstimate leftLL) {

        Pose2d currentOdometryPose = drivetrain.getState().Pose;
        double currentSpinRate = Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond);


        if (isValidVisionMeasurement(rightLL)) {
            Matrix<N3, N1> stdDevs = calculateStdDevs(rightLL, currentOdometryPose, currentSpinRate );
            updateRobotPose(drivetrain, rightLL, stdDevs, LimelightConstants.RIGHT_CAMERA_NAME);
        }

        if (isValidVisionMeasurement(leftLL)) {
            Matrix<N3, N1> stdDevs = calculateStdDevs(leftLL, currentOdometryPose, currentSpinRate );      
            updateRobotPose(drivetrain, leftLL, stdDevs, LimelightConstants.LEFT_CAMERA_NAME);    
        }
    }

  private void updateRobotPose(
    CommandSwerveDrivetrain drivetrain,
    LimelightHelpers.PoseEstimate mt2, 
    Matrix<N3, N1> stdDevs,
    String cameraName) {

    drivetrain.setVisionMeasurementStdDevs(stdDevs);
    drivetrain.addVisionMeasurement(
      mt2.pose,
      mt2.timestampSeconds
    ); 
  }

  private boolean isValidVisionMeasurement(LimelightHelpers.PoseEstimate mt2) {
    return mt2 != null && mt2.tagCount > 0;
  }

  /**
   * Dynamically calculates the standard deviation (trust) matrix based on area and tag count.
   */

  private Matrix<N3, N1> calculateStdDevs(
      LimelightHelpers.PoseEstimate estimate, 
      Pose2d currentOdometryPose,
      double currentSpinRate) {

      double xyStdDev = 9999999;
      double thetaStdDev = 9999999;
      
      double jumpDistance = currentOdometryPose.getTranslation().getDistance(estimate.pose.getTranslation());
      
      if (estimate.tagCount >= 2) {
          // Multi-tag tracking eliminates ambiguity, so we trust it much more than single-tag.
          // However, we still degrade trust as the average distance to the tags increases.
          
          double avgDist = estimate.avgTagDist;

          // Base StdDev (0.1 is very trusted) + (Distance Squared * Small Scaling Factor)
          // E.g., at 1 meter: 0.1 + (1 * 0.02) = 0.12 (Extremely confident)
          // E.g., at 5 meters: 0.1 + (25 * 0.02) = 0.60 (Less confident, rely more on wheel odometry)
          xyStdDev = 0.1 + (Math.pow(avgDist, 2) * 0.02); 
          
          // Heading is very stable with multiple tags, but still scales with distance
          thetaStdDev = 0.1 + (Math.pow(avgDist, 2) * 0.02); 
      } else {
          double tagArea = estimate.rawFiducials[0].ta;

          boolean isLargeTag = tagArea > 0.015; // You can tune this percentage

          // If it's a large tag OR the jump is small, AND we aren't spinning too fast
          if ((isLargeTag || jumpDistance <= 1.0) && currentSpinRate < (4.0 * Math.PI)) {
              
              if (tagArea > 0.0) {
                  xyStdDev = 0.5 + (0.005 / tagArea);
              } 
            
              thetaStdDev = 9999999; 
          }
      }

      return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }
}
