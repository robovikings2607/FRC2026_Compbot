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

        // STEP 2: Get the Initial Guess for Time of Flight
        double estimatedToF = getTargetState(realDistance).timeOfFlightSeconds;

        // STEP 3: Calculate the Virtual Target
        // We shift the target in the OPPOSITE direction of our travel.
        // If we drive +X, the ball carries that +X momentum, so we must aim -X to compensate.
        double virtualTargetX = realTargetPosition.getX() - (fieldRelVelocity.vxMetersPerSecond * estimatedToF);
        double virtualTargetY = realTargetPosition.getY() - (fieldRelVelocity.vyMetersPerSecond * estimatedToF);
        Translation2d virtualTarget = new Translation2d(virtualTargetX, virtualTargetY);

        // STEP 4: Calculate the Virtual Distance
        // How far away is this new, shifted target from our robot?
        double virtualDistance = robotPosition.getDistance(virtualTarget);

        // STEP 5: Get the Final Shooter State using the Virtual Distance
        // This gives us the perfectly corrected Hood Angle and Flywheel RPM!
        ShooterState finalState = getTargetState(virtualDistance);

        // STEP 6: Package it up and return it
        return new AimingSolution(virtualTarget, finalState);
    }
    
}