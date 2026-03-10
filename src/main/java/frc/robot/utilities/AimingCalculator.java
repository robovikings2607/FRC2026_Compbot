package frc.robot.utilities;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


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
            Translation2d robotPosition, 
            Translation2d realTargetPosition, 
            ChassisSpeeds fieldRelVelocity) {

        // STEP 1: Calculate the Real Distance
        double realDistance = robotPosition.getDistance(realTargetPosition);

        // STEP 2: Get the BASE State using REAL Distance
        // This guarantees your Hood Angle is perfectly matched to your physical location
        ShooterState baseState = getTargetState(realDistance);
        double estimatedToF = baseState.timeOfFlightSeconds;

        // STEP 3: Calculate the Virtual Target (For Turret/Heading)
        // This shifts the target to compensate for lateral (sideways) drift
        double virtualTargetX = realTargetPosition.getX() - (fieldRelVelocity.vxMetersPerSecond * estimatedToF);
        double virtualTargetY = realTargetPosition.getY() - (fieldRelVelocity.vyMetersPerSecond * estimatedToF);
        Translation2d virtualTarget = new Translation2d(virtualTargetX, virtualTargetY);

        // STEP 4: Calculate Radial Velocity using a Dot Product
        // We need to find out how much of our chassis speed is pointed directly at the target.
        double dx = realTargetPosition.getX() - robotPosition.getX();
        double dy = realTargetPosition.getY() - robotPosition.getY();
        
        // Normalize the vector (create a unit vector pointing at the target)
        double unitX = dx / realDistance;
        double unitY = dy / realDistance;

        // Dot product: (Velocity X * Unit X) + (Velocity Y * Unit Y)
        // If positive: Robot is moving TOWARD the goal.
        // If negative: Robot is moving AWAY from the goal (fadeaway).
        double radialVelocityMPS = (fieldRelVelocity.vxMetersPerSecond * unitX) + 
                                   (fieldRelVelocity.vyMetersPerSecond * unitY);

        // STEP 5: Calculate the RPS Compensation
        // You will need to experimentally tune this constant on the practice field.
        // "For every 1 meter per second of robot speed, alter the wheel by X RPM"
        double RPS_PER_MPS_FACTOR = 50.0 / 60; 
        
        // If moving toward (+), subtract RPM to avoid overshooting.
        // If moving away (-), subtracting a negative adds RPM to give the ball more energy.
        double compensatedRPS = baseState.rps - (radialVelocityMPS * RPS_PER_MPS_FACTOR);

        // STEP 6: Package the Hybrid State
        // We build a new state combining the perfect arc (Hood) with the compensated energy (RPM)
        ShooterState finalCompensatedState = new ShooterState(
            compensatedRPS, 
            baseState.hoodAngle, // Strictly uses the real distance arc!
            baseState.timeOfFlightSeconds
        );

        return new AimingSolution(virtualTarget, finalCompensatedState);
    }    
}