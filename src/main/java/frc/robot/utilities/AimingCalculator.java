package frc.robot.utilities;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;


public class AimingCalculator {
    
    private final InterpolatingTreeMap<Double, ShooterState> shotMap = 
        new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), 
            ShooterState::interpolate
    );

    public AimingCalculator() {
        // Populate your tuned distances here
        shotMap.put(2.0, new ShooterState(45.0, 30.0));
        shotMap.put(3.0, new ShooterState(52.0, 40.0));
        shotMap.put(4.5, new ShooterState(60.0, 48.0));
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
}