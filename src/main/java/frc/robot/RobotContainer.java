// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DoNothing;
//import frc.robot.commands.drivetrain.ToggleFieldCentric;
import frc.robot.commands.drivetrain.ToggleHighLowGear;
import frc.robot.commands.intake.ReverseRollers;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.DeployIntakeAndRunKicker;
import frc.robot.commands.intake.ForceIntakeDown;
import frc.robot.commands.intake.JostlePieces;
import frc.robot.commands.intake.KickerDefaultCommand;
import frc.robot.commands.intake.PIDTuningIntake;
import frc.robot.commands.intake.PulseKicker;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.intake.RetratctIntakeAndStopKicker;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.commands.shooter.ToggleOperatorControls;
import frc.robot.commands.shooter.ActivateTurret;
import frc.robot.commands.shooter.AutonShoot;
import frc.robot.commands.shooter.DeactivateTurret;
import frc.robot.commands.shooter.FixShooter;
import frc.robot.commands.shooter.ReverseSpindexer;
import frc.robot.commands.shooter.Shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.utilities.AxisButton;
import frc.robot.utilities.ISysIdTunable;
import frc.robot.utilities.RobotLogger;

public class RobotContainer {
    public Field2d field = new Field2d();

    private double MaxSpeed = 1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
         /* Request for driving in Robotcentric mode */
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private boolean fieldCentricDrive = true;
    private boolean lowGear = true;
    private double highGear = 1.0; // 0.6 or 60% (max speed) for competition
    private double slowGear = 0.4; // 0.2 or 20% of max speed for competition
    private double driveSpeedScale = (lowGear ? slowGear : highGear);
    public OI driverController = new OI(OI.kDriverControllerPort);
    public OI operatorController = new OI(OI.kOperatorControllerPort);
    
    public boolean safeToMove = false;
    private SendableChooser<Command> autoChooser;

    private boolean fixedShot = false;
    private boolean operatorEnabled = false;
    
    // Slew rate limiters for shoot-on-the-move — limit acceleration while shooting
    // Units: m/s per second for translation, rad/s per second for rotation
    private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(4.0);
    private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(4.0);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Math.PI * 2);
    private Shoot shootCommand;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final LimelightSubsystem limelight = new LimelightSubsystem(this);
    public final TurretSubsystem turret = new TurretSubsystem(this);
    public final FlywheelSubsystem flywheel = new FlywheelSubsystem(this);
    public final HoodSubsystem hood = new HoodSubsystem(this);
    public final FeederSubsystem feeder = new FeederSubsystem(this);
    public final SpindexerSubsystem spindexer = new SpindexerSubsystem(this);
    public final IntakeSubsystem intake = new IntakeSubsystem(this);
    public final KickerSubsystem kicker = new KickerSubsystem(this);
    //public final LEDSubsystem leds = new LEDSubsystem(this);

    public RobotContainer() {

        configureBindings();
        configureNamedCommands();
        createTuningToggles();
        //configureSysIdBindings(drivetrain);
        
        kicker.setDefaultCommand(new KickerDefaultCommand(this));

        autoChooser = AutoBuilder.buildAutoChooser();
        
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Field!!!", field);
    }

    private void configureBindings() {
 
        RobotLogger.logBoolean("FieldCentricMode", fieldCentricDrive);
        RobotLogger.logBoolean("HighGear", !lowGear);
        RobotLogger.logBoolean("OperatorEnabled", operatorEnabled);

        // Initialize the swerve drive to be controlled by the driver's controller
        setDrivetrainMode();

        // reset the field-centric heading on left bumper press
        driverController.startButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // toggle between field-centric and robot-centric drive
        // driverController.buttonY.onTrue(new ToggleFieldCentric(this));

        // toggle between hi gear and low gear
        driverController.rightStick.onTrue(new ToggleHighLowGear(this));

        //Intake
        driverController.leftTriggerButton.onTrue(new DeployIntake(this)); //will be deploy later
        driverController.leftBumper.onTrue(new RetractIntake(this)); //will be retract later
        driverController.rightBumper.onTrue(new ReverseRollers(this));
        driverController.buttonA.whileTrue(drivetrain.applyRequest(() -> brake));

        //Shooter
        //driverController.buttonA.onTrue(new PIDTuningIntake(this));
        shootCommand = new Shoot(this);
        driverController.rightTriggerButton.whileTrue(shootCommand);
        RobotLogger.logBoolean("ShootCommandSchedules", shootCommand.isScheduled());
        //driverController.buttonB.onTrue(new DeactivateTurret(this));
        //driverController.buttonX.onTrue(new ActivateTurret(this));
        //driverController.buttonY.onTrue(new FixShooter(this));

        //Operator/Emergency
        operatorController.backButton.and(operatorController.startButton).onTrue(new ToggleOperatorControls(this));
        operatorController.buttonY.onTrue(new RetractIntake(this));
        operatorController.buttonA.onTrue(new DeployIntake(this));
        operatorController.buttonB.onTrue(new DeactivateTurret(this));
        operatorController.buttonX.onTrue(new ActivateTurret(this));
        operatorController.leftBumper.onTrue(new JostlePieces(this));
        operatorController.leftTriggerButton.onTrue(new FixShooter(this));
        //operatorController.rightBumper.whileTrue(new ReverseSpindexer(this));
        operatorController.rightTriggerButton.onTrue(new ForceIntakeDown(this));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize2);
    }

    public void configureNamedCommands(){
        //PathPlanner Commands
        NamedCommands.registerCommand("DoNothing", new DoNothing());
        NamedCommands.registerCommand("DeployIntake", new DeployIntakeAndRunKicker(this).raceWith(new WaitCommand(0.1)));
        NamedCommands.registerCommand("DeployIntakeNoSOTM", new DeployIntakeAndRunKicker(this));
        NamedCommands.registerCommand("RetractIntake", new RetratctIntakeAndStopKicker(this));
        NamedCommands.registerCommand("Shoot", (new AutonShoot(this).raceWith(new WaitCommand(7.0))));
        NamedCommands.registerCommand("ShootOnTheMove", new Shoot(this));
    }

    public void createTuningToggles(){
        SmartDashboard.putBoolean("Tuning/EnablePIDTuning", false);
        SmartDashboard.putBoolean("Tuning/EnableDistanceTuning", true);
    }
     /**
     * VERY IMPORTANT: Put the robot up on blocks before running these!
     * VERY IMPORTANT: Change the button bindings to those you want to trigger the 4 tests
     * 
     * Configure buttons to run a SysId test against a particular subsystem's motor to get
     * the best values for that motor's PID configuration.
     * 
     * You always want to do this outside of the normal context of running the robot
     * only call this method from the constructor of RobotContainer and comment out the rest
     * of the code in that constructor
     * 
     * @param tuningTarget  The subsystem implementing the ISysIdTunable interface
     *                      whose motor you want to run the Sysid tool against
     */
    private void configureSysIdBindings(ISysIdTunable tuningTarget) {
        
        SysIdRoutine routineToTune = tuningTarget.getSysIdRoutine(); 

        //Quasistatic Forward
        operatorController.buttonA.whileTrue(
            routineToTune.quasistatic(SysIdRoutine.Direction.kForward)
        );
        
        //Quasistatic Reverse
        operatorController.buttonB.whileTrue(
            routineToTune.quasistatic(SysIdRoutine.Direction.kReverse)
        );
        
        //Dynamic Forward
        operatorController.buttonX.whileTrue(
            routineToTune.dynamic(SysIdRoutine.Direction.kForward)
        );
        
        //Dynamic Reverse
        operatorController.buttonY.whileTrue(
            routineToTune.dynamic(SysIdRoutine.Direction.kReverse)
        );
    }

    // Toggle low gear and high gear speeds
    public void toggleHiLoGear() {
        lowGear = !lowGear;
        driveSpeedScale = (lowGear ? slowGear : highGear);
        RobotLogger.logBoolean("HighGear", !lowGear);
    }

    public void setLowGear(boolean lowGear){
        this.lowGear = lowGear;
        driveSpeedScale = (lowGear ? slowGear : highGear);
                RobotLogger.logBoolean("HighGear", !lowGear);


    }

    public boolean isLowGear(){
        return lowGear;
    }

    public void toggleFixedShot(){
        fixedShot = !fixedShot;
    }

    public boolean isFixedShot(){
        return fixedShot;
    }

    public void toggleOperatorControls(){
        operatorEnabled = !operatorEnabled;
    }

    public boolean operatorEnabled(){
        return operatorEnabled;
    }

    // Returns 0 if reading the alliance color fails
    // Returns 1 if the alliance is Red
    // Returns -1 if the alliance is Blue
    public int readAllianceSign() {
        int allianceSign = 0;

        Optional<Alliance> myAlliance = DriverStation.getAlliance(); // Get alliance information

        if (myAlliance.isPresent()) {
            if (myAlliance.get() == Alliance.Red) {
                allianceSign = 1;
            }
            if (myAlliance.get() == Alliance.Blue) {
                allianceSign = -1;
            }
        }

        if ((allianceSign == 0) && !RobotBase.isReal()) {
            allianceSign = Constants.Simulation.simAlliance;
        }

        RobotLogger.logDouble("AllianceSign", allianceSign);

        return allianceSign;
    }

     // Toggle the swerve drive mode between field centric and robot centric
    // If it is field centric, change it to robot centric
    // If it is robot centric, change it to field centric
    public void toggleDriveMode() {
        setFieldCentric(!fieldCentricDrive);
        RobotLogger.logBoolean("FieldCentricMode", fieldCentricDrive);
        // setDrivetrainMode();
    }

     // Set the swerve mode drive to field centric (true) or robot centric (false)
    public void setFieldCentric(boolean state) {
        fieldCentricDrive = state;
        RobotLogger.logBoolean("FieldCentricMode", fieldCentricDrive);
        setDrivetrainMode();
    }

     // Return the swerve drive mode. True for field centric and false for robot
    // centric.
    public boolean getFieldCentric() {
        return fieldCentricDrive;
    }

    private double slewIfShooting(double input, SlewRateLimiter limiter) {
        return input;
        /* 
        if (shootCommand.isScheduled()) {
            return limiter.calculate(input);
        }
        limiter.reset(input);
        return input;
        */
    }

     // This method is called to modify the drivetrain mode
    private void setDrivetrainMode() {
        if (getFieldCentric()) {
            drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> driveFieldCentric
                .withVelocityX(slewIfShooting(-driverController.controller.getLeftY() * MaxSpeed * driveSpeedScale, m_xLimiter)) // Drive forward with negative Y (forward)
                .withVelocityY(slewIfShooting(-driverController.controller.getLeftX() * MaxSpeed * driveSpeedScale, m_yLimiter)) // Drive left with negative X (left)
                .withRotationalRate(slewIfShooting(-driverController.controller.getRightX() * MaxAngularRate * driveSpeedScale, m_rotLimiter)) // Drive counterclockwise with negative X (left)
            ));
        } else {
            drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                    drivetrain.applyRequest(() -> driveRobotCentric
                    .withVelocityX(slewIfShooting(-driverController.controller.getLeftY() * MaxSpeed * driveSpeedScale, m_xLimiter)) // Drive forward with negative Y (forward)
                    .withVelocityY(slewIfShooting(-driverController.controller.getLeftX() * MaxSpeed * driveSpeedScale, m_yLimiter)) // Drive left with negative X (left)
                    .withRotationalRate(slewIfShooting(-driverController.controller.getRightX() * MaxAngularRate * driveSpeedScale, m_rotLimiter)) // Drive counterclockwise with negative X (left)
                    ));
        }
    }

    
    public Command getAutonomousCommand() {
       return autoChooser.getSelected();
            
    }
    
public static class OI {
  public final XboxController controller;
  public final JoystickButton buttonA, buttonB, buttonY, buttonX, startButton, screenShotButton,
                              backButton, rightBumper, leftBumper, rightStick, leftStick;
  public final POVButton povNorth, povEast, povSouth, povWest;
  public AxisButton rightTriggerButton, leftTriggerButton;

  public static final int kDriverControllerPort = 1;
  public static final int kOperatorControllerPort = 0;

  
  public OI(int constant){
    controller = new XboxController(constant);
    // init buttons
    buttonA = new JoystickButton(controller, Button.kA.value);
    buttonB = new JoystickButton(controller, Button.kB.value);
    buttonX = new JoystickButton(controller, Button.kX.value);
    buttonY = new JoystickButton(controller, Button.kY.value);
    //
    startButton = new JoystickButton(controller, Button.kStart.value);
    backButton = new JoystickButton(controller, Button.kBack.value);
    screenShotButton = new JoystickButton(controller, 11);
    //
    povNorth = new POVButton(controller, 0);
    povEast = new POVButton(controller, 90);
    povSouth = new POVButton(controller, 180);
    povWest = new POVButton(controller, 270);
    //
    rightBumper = new JoystickButton(controller, Button.kRightBumper.value);
    leftBumper = new JoystickButton(controller, Button.kLeftBumper.value);
    //
    rightStick = new JoystickButton(controller, Button.kRightStick.value);
    leftStick = new JoystickButton(controller, Button.kLeftBumper.value);
    //
    rightTriggerButton = new AxisButton(controller, 3, .5);
    leftTriggerButton = new AxisButton(controller, 2, .5);
  }
}

}