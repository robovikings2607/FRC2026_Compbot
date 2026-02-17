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
    Rotation2d zeroMotorPositionAngleDegrees) {

    Translation2d offsetVector = new Translation2d(offsetDistanceMeters, offsetAngleDegrees);
    Transform2d robotToOffset = new Transform2d(offsetVector, zeroMotorPositionAngleDegrees);
    Pose2d offsetPose = startingPoint.transformBy(robotToOffset);    
    
    return offsetPose;    
  }  

  public static double getTargetAngleDegrees(Translation2d source, Translation2d target) {
      Translation2d offset = target.minus(source);
      return Math.toDegrees(Math.atan2(offset.getY(), offset.getX()));
  }

  public static double getTargetDistance(Translation2d source, Translation2d target) {
      Translation2d offset = target.minus(source);
      return Math.hypot(offset.getX(), offset.getY());
  }

    public static double getShortestPathToTargetDegrees(double targetDegrees, double currentDegrees) {
    return Math.IEEEremainder(targetDegrees - currentDegrees, 360.0);
  }

  public static double getMotorRotationsAsDegrees(double motorRotations, double gearRatio) {
    return (motorRotations / gearRatio) * 360.0;    
  }

  public static double getDegreesAsMotorRotations(double degrees , double gearRatio) {
    return (degrees / 360.0) * gearRatio;
  }

  public static double getAdjustedMechanismAngleDegrees(double mechanismAngleDegrees, double robotAngleDegrees) {
    return mechanismAngleDegrees - robotAngleDegrees;
  }

}
