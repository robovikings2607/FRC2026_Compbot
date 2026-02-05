package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ToggleFieldCentric extends Command {

  private RobotContainer robot;

  /** Creates a new ToggleFieldCentric. */
  public ToggleFieldCentric(RobotContainer robot) {
    this.robot = robot;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robot.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robot.toggleDriveMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}