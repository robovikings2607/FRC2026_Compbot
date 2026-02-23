package frc.robot.utilities;

import edu.wpi.first.math.geometry.Translation2d;

// This holds the complete set of instructions for the entire aiming system
public record AimingSolution(
    Translation2d virtualTarget, 
    ShooterState shooterState
) {}