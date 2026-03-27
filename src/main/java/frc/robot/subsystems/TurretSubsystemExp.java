// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.TurretConstants;
import frc.robot.utilities.GeometryUtil;
import com.ctre.phoenix6.controls.VoltageOut;


public class TurretSubsystemExp extends ShooterComponentSubsystemExp {
  private static final double GEAR_RATIO = 10.0; // 10 motor turns = 1 turret turn  
  private static final double BOT_TO_TURRET_DISTANCE_INCHES = Math.sqrt(Math.pow(0, 2) + Math.pow(7, 2));
  private static final double BOT_TO_TURRET_ANGLE_DEGREES = Math.toDegrees(Math.atan2(7, 0));
  private static final double ZERO_MOTOR_POSITION_ANGLE_DEGREES = 0;  
  private final double TARGET_ERR_TOLERANCE_ROTATIONS = 0.01;    
  private double unclampedTargetAngleDegrees = 0.0;
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  
  
  public TurretSubsystemExp(RobotContainer robot) {
    super(robot, TurretConstants.TURRET_ID);    

    configureMotor();

    SmartDashboard.putNumber("Turret/MotorCurrent", motor.getStatorCurrent().getValueAsDouble());
  }


  @Override
  public void periodic() {

  }

  /**
   * Aims the physical turret at a specific coordinate on the field.
   * @param fieldRelativeTarget The X/Y field coordinates of the target (Real or Virtual)
   * @param robotPose           The current field pose of the drivetrain
   */
  public void trackTarget(Translation2d fieldRelativeTarget, Pose2d robotPose) {
      
    Translation2d turretFieldPosition = getTurretFieldPosition(robotPose);

    // 2. Vector Math: Calculate the line from the Turret to the Target
    Translation2d turretToTargetVector = fieldRelativeTarget.minus(turretFieldPosition);

    Rotation2d fieldAngleToTarget = turretToTargetVector.getAngle();

    Rotation2d robotRelativeAngle = fieldAngleToTarget.minus(robotPose.getRotation());

    Rotation2d motorTargetAngle = robotRelativeAngle.minus(Rotation2d.fromDegrees(ZERO_MOTOR_POSITION_ANGLE_DEGREES));

    double optimizedTargetDegrees = MathUtil.inputModulus(
        motorTargetAngle.getDegrees(), 
        -180.0, 
        180.0
    );

    if (optimizedTargetDegrees < TurretConstants.MIN_ANGLE) {
      optimizedTargetDegrees += 360;
    }

    unclampedTargetAngleDegrees = optimizedTargetDegrees; 

    // Clamp the setpoint to your physical soft limits 
    double safelyClampedDegrees = MathUtil.clamp(optimizedTargetDegrees, TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE);

    double targetRotations = GeometryUtil.getDegreesAsMotorRotations(safelyClampedDegrees, GEAR_RATIO);      
   
    SetAgressiveMotorPosition(targetRotations, "Turret/newSetPointRotations");
  }

  public double getUnclampedTargetAngleDegrees() {
    return unclampedTargetAngleDegrees;
  }

  public Translation2d getTurretFieldPosition(Pose2d robotPose) {
    return getTurretPose(robotPose).getTranslation();
  }

  public Pose2d getTurretPose(Pose2d robotPose) {

    Pose2d turretPose = GeometryUtil.getOffsetPose(
      robotPose,
      Units.inchesToMeters(BOT_TO_TURRET_DISTANCE_INCHES),
      Rotation2d.fromDegrees(BOT_TO_TURRET_ANGLE_DEGREES)
    );    

    return turretPose;
  }


  /**
   * Checks if the turret is ready to fire.
   * @param currentRobotPose The current estimated pose of the robot (Odometry).
   * @param targetCoordinates The position of the target.
   * @param robotVelocity The current speed of the chassis (Linear + Angular).
   * @return True if safe to fire, False if aiming or unstable.
   */
  public boolean isReadyToShoot(double idealTargetDegrees, ChassisSpeeds robotVelocity) {
      
      // --- 2. THE REALITY (Actual Turret State) ---
      // Get the current position and velocity from the motor.
      // Convert rotations to degrees if your motor API uses rotations.
      double actualDegrees = GeometryUtil.getMotorRotationsAsDegrees(
        motor.getPosition().getValueAsDouble(), GEAR_RATIO);

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

      double currentToleranceDegrees = isRobotMoving ? 3.0 : 1.0;
      
      // --- 6. STABILITY CHECK (Velocity Interlock) ---
      // Even if error is low, don't fire if we are "sweeping" past the target.
      // We want the turret to be relatively still (tracked).
      // Max allowable sweep speed: ~10 degrees per second.
      double turretVelocityDegPerSec = motor.getVelocity().getValueAsDouble() * 360.0;
      boolean isStable = Math.abs(turretVelocityDegPerSec) < 10.0;

      // --- 7. FINAL DECISION ---
      return (error < currentToleranceDegrees) && isStable;
  }  

  @Override
  protected void configureMotor(){

    TalonFXConfiguration configs = new TalonFXConfiguration();

    var slot0Configs = configs.Slot0;
        slot0Configs.kS = 0.25; // Voltage output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps requires this voltage output.
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires this voltage output
        slot0Configs.kP = 20.0; // A position error of .55 rotations requires this voltage output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.5; // A velocity error of .5 rps requires this voltage output

    var slot1Configs = configs.Slot1; // Use Slot 1 for Tracking!
      slot1Configs.kS = 0.25;        
      slot1Configs.kV = 0.12; // Feedforward is critical for tracking
      slot1Configs.kP = 40.0; // Much stiffer! (Example value, must tune)
      slot1Configs.kI = 0.0;
      slot1Configs.kD = 2.0;  // High damping to stop the shake

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
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = GeometryUtil.getMotorRotationsAsDegrees(TurretConstants.MAX_ANGLE, GEAR_RATIO);
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = GeometryUtil.getMotorRotationsAsDegrees(TurretConstants.MIN_ANGLE, GEAR_RATIO);;


    motor.getConfigurator().apply(configs);
    motor.setNeutralMode(NeutralModeValue.Brake);

    //Note, this is dangerous as it only leads the turret to believe it's at 0 even
    //though it may physically be at a different angle. Should set initial position 
    //based on some absolute source if available.
    motor.setPosition(0); //
  }

  @Override
  public double getTargetTolerance() {
    return TARGET_ERR_TOLERANCE_ROTATIONS;
  }

public void applyRawVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
}

public double getTurretStatorCurrent() {
    return motor.getStatorCurrent().getValueAsDouble();
}

public void setEncoderToAngle(double degrees) {
    double motorRotations = GeometryUtil.getDegreesAsMotorRotations(degrees,GEAR_RATIO);
    
    motor.setPosition(motorRotations);
}

}
