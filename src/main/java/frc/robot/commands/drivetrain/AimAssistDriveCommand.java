package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.OI;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystemExp;

public class AimAssistDriveCommand extends Command {

  private RobotContainer robot;
    private final CommandSwerveDrivetrain drivetrain;
    private final TurretSubsystemExp turret;
    private final OI driverController;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    // The Automated Unwind Logic
    private final PIDController unwindPID = new PIDController(0.05, 0.0, 0.0);
    private static final double ASSIST_THRESHOLD_DEGREES = 120.0;
    

  /** Creates a new ToggleFieldCentric. */
    public AimAssistDriveCommand(
            CommandSwerveDrivetrain drivetrain, 
            TurretSubsystemExp turret, 
            OI driverController) {
        
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.driverController = driverController;

        // CRITICAL: This tells WPILib to interrupt the default drive command!
        addRequirements(this.drivetrain); 
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
        // 1. Keep the driver's X/Y translation exacty the same
        double translationX = -robot.driverController.controller.getLeftY() * robot.MaxSpeed * robot.driveSpeedScale;
        double translationY = -robot.driverController.controller.getLeftX() * robot.MaxSpeed * robot.driveSpeedScale;
        
        // 2. Calculate the automated rotation
        double automatedRotationalRate = 0.0;
        double turretTarget = robot.turretExp.getUnclampedTargetAngleDegrees();

        // 3. Does the Turret need help?
        if (Math.abs(turretTarget) > ASSIST_THRESHOLD_DEGREES) {
            // Push the chassis to unwind the turret back to 0
            double pidOutput = unwindPID.calculate(turretTarget, 0.0);
            automatedRotationalRate = MathUtil.clamp(pidOutput, -robot.MaxAngularRate, robot.MaxAngularRate);
        } else {
            // Turret is safe. Force chassis rotation to 0 to keep it stable.
            // (Or you could read driverController.getRightX() here if you want 
            // the driver to still be able to spin manually while aiming).
            //automatedRotationalRate = 0.0;
            automatedRotationalRate = -driverController.controller.getRightX() * robot.MaxAngularRate;
        }

        driveRequest
            .withVelocityX(translationX)
            .withVelocityY(translationY)
            .withRotationalRate(automatedRotationalRate);

        this.drivetrain.setControl(driveRequest);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}