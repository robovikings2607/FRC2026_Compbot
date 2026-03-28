package frc.robot.utilities;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.utilities.AimingSolution;


public class AimingCalculator {
    
    private final InterpolatingTreeMap<Double, ShooterState> shotMap = 
        new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), 
            ShooterState::interpolate
    );

    public AimingCalculator() {
        // Populate your tuned distances here
        shotMap.put(2.0, new ShooterState(45.0, 30.0, 0.35));
        shotMap.put(3.0, new ShooterState(52.0, 40.0, 0.45));
        shotMap.put(4.5, new ShooterState(60.0, 48.0, 0.55));
    }

    /**
     * Calculates the required shooter state for a given distance.
     */
    public ShooterState getTargetState(double distanceMeters) {
        // Add a safety clamp so we don't request a shot from 50 feet away
        if (distanceMeters > 5.0) {
            return shotMap.get(5.0); // Return max range
        }
        return shotMap.get(distanceMeters);
    }

    /**
     * Calculates the complete aiming solution for shooting while moving.
     * @param robotPosition     The true field position of the Turret (X, Y)
     * @param realTargetPosition The true field position of the Speaker (X, Y)
     * @param fieldRelVelocity  The robot's field-relative velocity vector (Vx, Vy in m/s)
     * @return An AimingSolution containing the Virtual Target and the final Shooter State.
     */
public AimingSolution calculateMovingAimingSolution(
      Pose2d robotPose, 
      Translation2d realTargetPosition, 
      ChassisSpeeds fieldRelVelocity) {
      
      Translation2d robotPosition = robotPose.getTranslation();

      // ==========================================
      // PART 1: THE PHYSICS (Flywheel & Hood)
      // ==========================================
      
      // 1. Calculate Real Physical Distance
      double realDistance = robotPosition.getDistance(realTargetPosition);

      // 2. Get Base State using REAL Distance
      // This protects the hood angle so the physical arc is always correct.
      ShooterState baseState = getTargetState(realDistance);
      double timeOfFlight = baseState.timeOfFlightSeconds;

      // 3. Calculate Radial Velocity (Dot Product)
      // Finds out how much of our chassis speed is pointed directly at the target.
      double dx = realTargetPosition.getX() - robotPosition.getX();
      double dy = realTargetPosition.getY() - robotPosition.getY();
      double unitX = dx / realDistance;
      double unitY = dy / realDistance;

      double radialVelocityMPS = (fieldRelVelocity.vxMetersPerSecond * unitX) + 
                                 (fieldRelVelocity.vyMetersPerSecond * unitY);

      // 4. Compensate Flywheel Speed
      // If moving toward (+), subtract RPM. If moving away (-), add RPM.
      double RPS_PER_MPS_FACTOR = 6.27; // Tune this on the field
      double compensatedRPS = baseState.rps - (radialVelocityMPS * RPS_PER_MPS_FACTOR);


      // ==========================================
      // PART 2: THE GEOMETRY (Turret Aiming)
      // ==========================================
      
      // 5. Calculate Corrective Vector
      // How far will the robot travel during the ball's flight time?
      Translation2d expectedTravel = new Translation2d(
          fieldRelVelocity.vxMetersPerSecond * timeOfFlight,
          fieldRelVelocity.vyMetersPerSecond * timeOfFlight
      );
      
      // We flip it because if the robot moves +Y, we must aim -Y to compensate.
      Translation2d correctiveVector = expectedTravel.unaryMinus();

      // 6. Shift the Target
      Translation2d correctedTarget = realTargetPosition.plus(correctiveVector);

      // 7. Calculate Turret Angle
      // Find the vector from the robot to the newly shifted target.
      Translation2d vectorToCorrectedTarget = correctedTarget.minus(robotPosition);
      
      // ==========================================
      // PART 3: PACKAGE AND RETURN
      // ==========================================
      
      ShooterState finalCompensatedState = new ShooterState(
          compensatedRPS, 
          baseState.hoodAngle, // Protected!
          timeOfFlight
      );

      // Returning the calculated heading for the turret, plus the physics for the shooter
      return new AimingSolution(vectorToCorrectedTarget, finalCompensatedState);
  }
}