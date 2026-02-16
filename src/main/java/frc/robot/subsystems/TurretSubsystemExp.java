// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.TurretConstants;
import frc.robot.utilities.GeometryUtil;

public class TurretSubsystemExp extends SubsystemBase {
  private static final double GEAR_RATIO = 10.0; // 10 motor turns = 1 turret turn  
  private static final double rotationsPerDegree = 10.0/360.0;
  private static final  double BOT_TO_TURRET_DISTANCE_INCHES = Math.sqrt(Math.pow(0, 2) + Math.pow(7, 2));
  private static final double BOT_TO_TURRET_ANGLE_DEGREES = Math.toDegrees(Math.atan2(7, 0));

  private final TalonFX turretMotor;
  private final RobotContainer robot;
  private final MotionMagicVoltage magicMotionRequest = new MotionMagicVoltage(0);
  private static final int MAX_TURRET_ANGLE_DEGREES = 162;
  private static final int MIN_TURRET_ANGLE_DEGREES = -162;  
  
  public TurretSubsystemExp(RobotContainer robot) {
    this.robot = robot;

    turretMotor = new TalonFX(TurretConstants.TURRET_ID);
    
    configureMotor();

    SmartDashboard.putNumber("Turret/MotorCurrent", turretMotor.getStatorCurrent().getValueAsDouble());
  }

  private void configureMotor() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    var slot0Configs = configs.Slot0;
        slot0Configs.kS = 0.25; // Voltage output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps requires this voltage output.
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires this voltage output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations requires this voltage output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.11; // A velocity error of 1 rps requires this voltage output

    var motionMagicConfigs = configs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 160; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 320; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)


    // 1. ENABLE SOFT LIMITS
    // The motor will stop if it tries to go past these values.
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    // 2. SET THE THRESHOLDS (in Rotations)
    // ALWAYS set these slightly safer than the physical hard stop.
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = rotationsPerDegree * MAX_TURRET_ANGLE_DEGREES;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = rotationsPerDegree * MIN_TURRET_ANGLE_DEGREES;

    turretMotor.getConfigurator().apply(configs);
    turretMotor.setNeutralMode(NeutralModeValue.Brake);

    //Note, this is dangerous as it only leads the turret to believe it's at 0 even
    //though it may physically be at a different angle. Should set initial position 
    //based on some absolute source if available.
    turretMotor.setPosition(0); //
    
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {

  }

  public void positionTurretOnTarget(
    Pose2d robotPose, 
    Translation2d targetCoordinates,
    ChassisSpeeds robotVelocity) {

    double idealTurretTargetAngleDegrees = getTurretAngleToTarget(robotPose, targetCoordinates, robotVelocity);

    //can't allow target angle to exceed the soft limits in either direction
    double safeTargetAngleDegrees = MathUtil.clamp(
      idealTurretTargetAngleDegrees, MIN_TURRET_ANGLE_DEGREES, MAX_TURRET_ANGLE_DEGREES);

    SmartDashboard.putNumber("Turret/newTurretAngleToTargetDegrees", safeTargetAngleDegrees);
    
    double finalMotorSetpointRotations = GeometryUtil.getDegreesAsMotorRotations(idealTurretTargetAngleDegrees, GEAR_RATIO);
    SmartDashboard.putNumber("Turret/newSetPointRotations", finalMotorSetpointRotations);
    
    turretMotor.setControl(magicMotionRequest.withPosition(finalMotorSetpointRotations));
  }  

    private double getTurretAngleToTarget(
      Pose2d robotPose, 
      Translation2d targetCoordinates, 
      ChassisSpeeds robotVelocity) {
    
      final double SHOT_VELOCITY_MPS = 10.0; // Speed of ball leaving shooter (Tune this!)

      Pose2d turretPose = GeometryUtil.getOffsetPose(
        robotPose,
        Units.inchesToMeters(BOT_TO_TURRET_DISTANCE_INCHES),
        Rotation2d.fromDegrees(BOT_TO_TURRET_ANGLE_DEGREES)
      );    

      // 2. Calculate Distance & Time of Flight (ToF)
      double distance = turretPose.getTranslation().getDistance(targetCoordinates);
      double timeOfFlight = distance / SHOT_VELOCITY_MPS;

      // 3. Calculate the "Virtual Target"
      // Concept: Aim at where the goal would be if it were moving opposite to us.
      // Formula: Target - (RobotVel * ToF)
      double robotVx = robotVelocity.vxMetersPerSecond;
      double robotVy = robotVelocity.vyMetersPerSecond;

      Translation2d driftAdjustment = new Translation2d(robotVx * timeOfFlight, robotVy * timeOfFlight);
      Translation2d virtualTarget = turretPose.getTranslation().minus(driftAdjustment);

      //calculate the angle from the turret center to the target center
      double turretTargetAngleDegrees = GeometryUtil.getTargetAngleDegrees(
        turretPose.getTranslation(), 
        virtualTarget);

      //adjust the target angle by the current rotation of the robot chassis 
      double robotRotationDegrees = MathUtil.inputModulus(robot.drivetrain.getState().Pose.getRotation().getDegrees(),-180, 180);
      double adjustedTurretTargetAngleDegrees = GeometryUtil.getAdjustedMechanismAngleDegrees(
        turretTargetAngleDegrees, 
        robotRotationDegrees);

      return adjustedTurretTargetAngleDegrees;
    }

  /**
   * Checks if the turret is ready to fire.
   * @param currentRobotPose The current estimated pose of the robot (Odometry).
   * @param targetCoordinates The position of the target.
   * @param robotVelocity The current speed of the chassis (Linear + Angular).
   * @return True if safe to fire, False if aiming or unstable.
   */
  public boolean isTurretReadyToFire(
    Pose2d currentRobotPose, 
    Translation2d targetCoordinates, 
    ChassisSpeeds robotVelocity) {
      
      // --- 1. THE TRUTH (Recalculate Fresh Target) ---
      // We re-calculate the angle right now to account for any robot lag.
      // NOTE: This must be the UNCLAMPED, ideal angle (-180 to 180).
      double idealTargetDegrees = getTurretAngleToTarget(currentRobotPose,  targetCoordinates, robotVelocity);
      
      // --- 2. THE REALITY (Actual Turret State) ---
      // Get the current position and velocity from the motor.
      // Convert rotations to degrees if your motor API uses rotations.
      double actualDegrees = turretMotor.getPosition().getValueAsDouble() * 360.0;

      // --- 3. THE ERROR (Distance to Truth) ---
      // Use inputModulus to handle the -180/180 wrap correctly.
      // Example: Target = 179, Actual = -179. Error should be 2, not 358.
      double error = Math.abs(MathUtil.inputModulus(idealTargetDegrees - actualDegrees, -180, 180));

      // --- 4. THE LIMIT CHECK (Implicit) ---
      // If Ideal is 160° (Blocked) and Actual is 135° (Hard Stop),
      // The error will be 25°. This naturally returns FALSE.
      
      // --- 5. DYNAMIC TOLERANCE (Compensate for Motion) ---
      // If the robot is driving fast, we open the window slightly.
      // If the robot is stopped, we demand perfection.
      // Check if robot is moving significantly (> 0.1 m/s or > 10 deg/s)
      boolean isRobotMoving = (Math.abs(robotVelocity.vxMetersPerSecond) > 0.1) || 
                              (Math.abs(robotVelocity.omegaRadiansPerSecond) > 0.2);

      double currentTolerance = isRobotMoving ? 3.0 : 1.0;
      
      // --- 6. STABILITY CHECK (Velocity Interlock) ---
      // Even if error is low, don't fire if we are "sweeping" past the target.
      // We want the turret to be relatively still (tracked).
      // Max allowable sweep speed: ~10 degrees per second.
      double turretVelocityDegPerSec = turretMotor.getVelocity().getValueAsDouble() * 360.0;
      boolean isStable = Math.abs(turretVelocityDegPerSec) < 10.0;

      // --- 7. FINAL DECISION ---
      return (error < currentTolerance) && isStable;
  }  
}
