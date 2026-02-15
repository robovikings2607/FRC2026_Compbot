package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;

public final class GeometryUtil {  
  public static double getTargetAngle(Translation2d source, Translation2d target) {
      Translation2d diff = target.minus(source);
      return Math.toDegrees(Math.atan2(diff.getY(), diff.getX()));
  }

  public static Pose2d getOffsetPose(
    Pose2d startingPoint, 
    double offsetDistanceMeters, 
    Rotation2d offsetAngleDegrees) {

    return getOffsetPose(startingPoint, offsetDistanceMeters, offsetAngleDegrees, new Rotation2d(0));    
  }

  public static Pose2d getOffsetPose(
    Pose2d startingPoint, 
    double offsetDistanceMeters, 
    Rotation2d offsetAngleDegrees,
    Rotation2d offsetBaseAngleDegrees) {

    Translation2d offsetVector = new Translation2d(offsetDistanceMeters, offsetAngleDegrees);
    Transform2d robotToOffset = new Transform2d(offsetVector, offsetBaseAngleDegrees);
    Pose2d offsetPose = startingPoint.transformBy(robotToOffset);    
    
    return offsetPose;    
  }  
}
