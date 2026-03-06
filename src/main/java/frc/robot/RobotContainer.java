// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.drivetrain.AimAssistDriveCommandExp;
import frc.robot.commands.DoNothing;
//import frc.robot.commands.drivetrain.ToggleFieldCentric;
import frc.robot.commands.drivetrain.ToggleHighLowGear;
import frc.robot.commands.intake.ReverseRollers;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.shooter.SpindexerShootCommandExp;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.AutoAimAndShootCommandExp;
import frc.robot.commands.shooter.TrackHubTargetExp;
import frc.robot.commands.shooter.TuneShooterCommandExp;
import frc.robot.commands.shooter.UnjamShooterCommandExp;
import frc.robot.commands.shooter.ZeroHoodCommandExp;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystemExp;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.FlywheelSubsystemExp;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HoodSubsystemExp;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.SpindexerSubsystemExp;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystemExp;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.utilities.AxisButton;

public class RobotContainer {
    public Field2d field = new Field2d();

    public final double MaxSpeed = 1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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
    private double slowGear = 0.3; // 0.2 or 20% of max speed for competition
    public double driveSpeedScale = (lowGear ? slowGear : highGear);
    public OI driverController = new OI(OI.kDriverControllerPort);
    public OI operatorController = new OI(OI.kOperatorControllerPort);
    
    public boolean safeToMove = false;
    private SendableChooser<Command> autoChooser;
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final LimelightSubsystem limelight = new LimelightSubsystem(this);

    //Subsystems
    //real
    public final TurretSubsystem turret = new TurretSubsystem(this);
    public final FlywheelSubsystem flywheel = new FlywheelSubsystem(this);
    public final HoodSubsystem hood = new HoodSubsystem(this);
    public final FeederSubsystem feeder = new FeederSubsystem(this);
    public final SpindexerSubsystem spindexer = new SpindexerSubsystem(this);
    public final IntakeSubsystem intake = new IntakeSubsystem(this);
    //public final LEDSubsystem leds = new LEDSubsystem(this);
    //public final ClimberSubsystem climber = new ClimberSubsystem(this);

    //experimental
    public final TurretSubsystemExp turretExp = new TurretSubsystemExp(this);
    public final HoodSubsystemExp hoodExp = new HoodSubsystemExp(this);
    public final FlywheelSubsystemExp flywheelExp = new FlywheelSubsystemExp(this);
    public final FeederSubsystemExp feederExp = new FeederSubsystemExp(this);    
    public final SpindexerSubsystemExp spindexerExp = new SpindexerSubsystemExp(this);        

    public RobotContainer() {

        //configureBindings();
        configureSysIdBindings();
        configureNamedCommands();
        configureDefaultCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Field!!!", field);
    }
    
    private void configureBindings() {
 
        SmartDashboard.putBoolean("FieldCentricMode", fieldCentricDrive);
        SmartDashboard.putBoolean("HighGear", !lowGear);

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

        //Shooter
        driverController.rightTriggerButton.whileTrue(new Shoot(this));
        driverController.buttonB.onTrue(new StopShooter(this));
        driverController.buttonY.onTrue(new ZeroHoodCommandExp(this).withTimeout(2.0));    
        driverController.buttonX.whileTrue(new UnjamShooterCommandExp(spindexerExp, feederExp)
);            

        //Operator/Emergency
        operatorController.buttonY.onTrue(new RetractIntake(this));
        operatorController.buttonA.onTrue(new DeployIntake(this));


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        configureShotCategorizationButtons();        

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureSysIdBindings() {
        // VERY IMPORTANT: Put the robot up on blocks before running these!
        
        // D-Pad Up: Quasistatic Forward (Slow ramp up)
        operatorController.buttonA.whileTrue(flywheelExp.sysIdQuasistaticForward());
        
        // D-Pad Down: Quasistatic Reverse (Slow ramp backward)
        operatorController.buttonB.whileTrue(flywheelExp.sysIdQuasistaticReverse());
        
        // D-Pad Right: Dynamic Forward (Instant jump to 7 volts)
        operatorController.buttonX.whileTrue(flywheelExp.sysIdDynamicForward());
        
        // D-Pad Left: Dynamic Reverse (Instant jump to -7 volts)
        operatorController.buttonY.whileTrue(flywheelExp.sysIdDynamicReverse());
    }

    private void configureShotCategorizationButtons() {
        // Y Button: Perfect Make
        driverController.buttonY.onTrue(
            new InstantCommand(() -> {
                SignalLogger.writeString("Scouting/ShotResult", "MAKE");
                System.out.println("Logged to Hoot: MAKE"); 
            })
        );

        // A Button: Missed Short
        driverController.buttonA.onTrue(
            new InstantCommand(() -> {
                SignalLogger.writeString("Scouting/ShotResult", "MISS_SHORT");
                System.out.println("Logged to Hoot: MISS_SHORT");
            })
        );

        // B Button: Missed Long
        driverController.buttonB.onTrue(
            new InstantCommand(() -> {
                SignalLogger.writeString("Scouting/ShotResult", "MISS_LONG");
                System.out.println("Logged to Hoot: MISS_LONG");
            })
        );
        
        // X Button: Missed Left/Right
        driverController.buttonX.onTrue(
            new InstantCommand(() -> {
                SignalLogger.writeString("Scouting/ShotResult", "MISS_WIDE");
                System.out.println("Logged to Hoot: MISS_WIDE");
            })
        );

    }

    public void configureNamedCommands(){
        //PathPlanner Commands
        NamedCommands.registerCommand("DoNothing", new DoNothing());
        NamedCommands.registerCommand("DeployIntake", new DeployIntake(this));
        NamedCommands.registerCommand("RetractIntake", new RetractIntake(this));
        NamedCommands.registerCommand("Shoot", new Shoot(this).raceWith(new WaitCommand(5.0)));
    }

    private void configureDefaultCommands() {
        turretExp.setDefaultCommand(new TrackHubTargetExp(
            turretExp, 
            () -> drivetrain.getState().Pose,
            this::getFieldRelativeVelocity
        ));

        flywheelExp.setDefaultCommand(new RunCommand(
            () -> flywheelExp.setRPS(FlywheelConstants.IDLE_RPM), 
            flywheelExp
        ));

        hoodExp.setDefaultCommand(new RunCommand(
            () -> hoodExp.setAngle(HoodConstants.ZERO_POSITION_ANGLE), 
            hoodExp
        ));
    }


    // Toggle low gear and high gear speeds
    public void toggleHiLoGear() {
        lowGear = !lowGear;
        driveSpeedScale = (lowGear ? slowGear : highGear);
        SmartDashboard.putBoolean("HighGear", !lowGear);
    }

    public boolean isLowGear(){
        return lowGear;
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

        SmartDashboard.putNumber("AllianceSign", allianceSign);

        return allianceSign;
    }

     // Toggle the swerve drive mode between field centric and robot centric
    // If it is field centric, change it to robot centric
    // If it is robot centric, change it to field centric
    public void toggleDriveMode() {
        setFieldCentric(!fieldCentricDrive);
        SmartDashboard.putBoolean("FieldCentricMode", fieldCentricDrive);
        // setDrivetrainMode();
    }

     // Set the swerve mode drive to field centric (true) or robot centric (false)
    public void setFieldCentric(boolean state) {
        fieldCentricDrive = state;
        SmartDashboard.putBoolean("FieldCentricMode", fieldCentricDrive);
        setDrivetrainMode();
    }

     // Return the swerve drive mode. True for field centric and false for robot
    // centric.
    public boolean getFieldCentric() {
        return fieldCentricDrive;
    }

     // This method is called to modify the drivetrain mode
    private void setDrivetrainMode() {
        if (getFieldCentric()) {
            drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> driveFieldCentric
                .withVelocityX(-driverController.controller.getLeftY() * MaxSpeed * driveSpeedScale) // Drive forward with negative Y (forward)
                .withVelocityY(-driverController.controller.getLeftX() * MaxSpeed * driveSpeedScale) // Drive left with negative X (left)
                .withRotationalRate(-driverController.controller.getRightX() * MaxAngularRate * driveSpeedScale) // Drive counterclockwise with negative X (left)
            ));
        } else {
            drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                    drivetrain.applyRequest(() -> driveRobotCentric
                    .withVelocityX(-driverController.controller.getLeftY() * MaxSpeed * driveSpeedScale) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.controller.getLeftX() * MaxSpeed * driveSpeedScale) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.controller.getRightX() * MaxAngularRate * driveSpeedScale) // Drive counterclockwise with negative X (left)
                    ));
        }
    }

    
    public Command getAutonomousCommand() {
       return autoChooser.getSelected();
            
    }
    
    /**
     * Gets the actual measured velocity of the robot in Field-Relative coordinates.
     * This is what you need for shooting on the move.
     */
    public ChassisSpeeds getFieldRelativeVelocity() {
        // 1. Get the current state of all modules (Velocity & Angle)
        // You likely already have a method or array for this to update Odometry.
        SwerveModuleState[] moduleStates = drivetrain.getModuleStates();

        // 2. Convert Module States -> Robot-Relative ChassisSpeeds
        // This tells us: "Robot is moving 2m/s Forward relative to its own nose."
        ChassisSpeeds robotRelativeSpeeds = drivetrain.getKinematics().toChassisSpeeds(moduleStates);

        // 3. Convert Robot-Relative -> Field-Relative
        // This tells us: "Robot is moving 1.5m/s North and 1m/s East."
        // We need the Gyroscope rotation for this math.
        
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds, drivetrain.getState().Pose.getRotation()); 
    }

    public static double calculateRotationOverride(double currentDegrees, double targetDegrees) {

        // A. The "Pin" Check (Are we physically stuck?)
        // We consider the turret "stuck" if it is within 5 degrees of the limit.
        // Example: 130 - 5 = 125.0
        double NEAR_LIMIT_POS = TurretConstants.MAX_ANGLE - 5.0;
        double NEAR_LIMIT_NEG = TurretConstants.MAX_ANGLE + 5.0; // -130 + 5 = -125

        // B. The "Trigger" (When to ask for help)
        // We only ask for help if the target is significantly PAST the limit.
        // This prevents the robot from jittering if the target is just barely out of reach.
        // Example: 130 + 5 = 135.0
        double TRIGGER_POS = TurretConstants.MAX_ANGLE + 5.0;
        double TRIGGER_NEG = TurretConstants.MAX_ANGLE - 5.0; // -130 - 5 = -135

        // --- 3. THE SPEED ---
        double UNWIND_SPEED = 2.0; // Rad/Sec

        // --- LOGIC: LEFT SIDE (Positive) ---
        // 1. Is the target asking us to go past the trigger point? (e.g. > 135)
        boolean targetIsForbiddenLeft = targetDegrees > TRIGGER_POS;
        
        // 2. Is the turret physically at the limit? (e.g. > 125)
        boolean turretIsPinnedLeft = currentDegrees > NEAR_LIMIT_POS;

        if (targetIsForbiddenLeft && turretIsPinnedLeft) {
            return UNWIND_SPEED; // Turn Left
        }

        // --- LOGIC: RIGHT SIDE (Negative) ---
        // 1. Is the target asking us to go past the trigger point? (e.g. < -135)
        boolean targetIsForbiddenRight = targetDegrees < TRIGGER_NEG;
        
        // 2. Is the turret physically at the limit? (e.g. < -125)
        boolean turretIsPinnedRight = currentDegrees < NEAR_LIMIT_NEG;

        if (targetIsForbiddenRight && turretIsPinnedRight) {
            return -UNWIND_SPEED; // Turn Right
        }

        return 0.0;

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