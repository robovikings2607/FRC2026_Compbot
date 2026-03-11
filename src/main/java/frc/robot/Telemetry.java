package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Telemetry {
    private final double MaxSpeed;
    private int telemetryLoopCounter = 0;

    // 1. Your manual toggle. Set to false before a match, true in the pits.
    public static boolean DEBUG_MODE = true;    
    
    // Grab the direct disk logger
    private static final DataLog log = DataLogManager.getLog();
    
    // Caches to hold our log entries so we don't recreate them every 20ms
    private static final Map<String, DoubleLogEntry> doubleLogs = new HashMap<>();
    private static final Map<String, BooleanLogEntry> booleanLogs = new HashMap<>();    

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        SignalLogger.start();

        /* Set up the module state Mechanism2d telemetry */
        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }

        /**
     * Logs data directly to the .wpilog file, and optionally to SmartDashboard.
     */
    public static void logDouble(String key, double value) {
        // 1. ALWAYS write directly to the disk (.wpilog)
        // We put it in a clean "/RobotData/" folder in AdvantageScope
        DoubleLogEntry entry = doubleLogs.computeIfAbsent(key, 
            k -> new DoubleLogEntry(log, "/RobotData/" + k));
        entry.append(value);

        // 2. ONLY push over Wi-Fi if it's safe and requested
        if (DEBUG_MODE && !DriverStation.isFMSAttached()) {
            SmartDashboard.putNumber(key, value);
        }
    }

    public static void logBoolean(String key, boolean value) {
        BooleanLogEntry entry = booleanLogs.computeIfAbsent(key, 
            k -> new BooleanLogEntry(log, "/RobotData/" + k));
        entry.append(value);

        if (DEBUG_MODE && !DriverStation.isFMSAttached()) {
            SmartDashboard.putBoolean(key, value);
        }
    }

    /**
     * Records data ONLY to the CTRE .hoot file for post-match analysis.
     * Ideal for high-frequency subsystem math (e.g., internal flywheel PID calculations)
     * to avoid clogging up Wi-Fi bandwidth.
     */
    public static void record(String key, double value) {
        SignalLogger.writeDouble(key, value);
    }

    public static void record(String key, boolean value) {
        SignalLogger.writeBoolean(key, value);
    }

    public static void record(String key, String value) {
        SignalLogger.writeString(key, value);
    }

    /**
     * Publishes data to SmartDashboard for live Driver Station viewing.
     * Keep this lean! Only send what the drive team or pit crew needs to see right now.
     */
    public static void publish(String key, double value) {
        SmartDashboard.putNumber(key, value);
    }

    public static void publish(String key, boolean value) {
        SmartDashboard.putBoolean(key, value);
    }

    public static void publish(String key, String value) {
        SmartDashboard.putString(key, value);
    }

    /**
     * Logs to BOTH live telemetry and the high-speed CTRE logger.
     * Use this for critical state variables, like a Turret's current target angle.
     */
    public static void logCritical(String key, double value) {
        publish(key, value);
        record(key, value);
    }
    
    public static void logCritical(String key, boolean value) {
        publish(key, value);
        record(key, value);
    }

    /**
     * Publishes data to SmartDashboard ONLY if DEBUG_MODE is true 
     * AND the robot is NOT connected to the official competition field.
     */
    public static void publishDebug(String key, double value) {
        if (DEBUG_MODE && !DriverStation.isFMSAttached()) {
            SmartDashboard.putNumber(key, value);
        }
    }

    public static void publishDebug(String key, boolean value) {
        if (DEBUG_MODE && !DriverStation.isFMSAttached()) {
            SmartDashboard.putBoolean(key, value);
        }
    }

    /**
     * Use this for critical match data that the drivers absolutely MUST see 
     * during a real match (like an Auto Chooser or "Shooter Ready" boolean).
     */
    public static void publishMatch(String key, double value) {
        SmartDashboard.putNumber(key, value);
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        // ==========================================
        // 1. HIGH FREQUENCY (250Hz)
        // ==========================================
        // Keep these running at full speed. Pose is critical for accurate 
        // path replay, and the others are lightweight thread diagnostics.
        drivePose.set(state.Pose);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);
        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");

        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        fieldPub.set(m_poseArray);


        // ==========================================
        // 2. LOW FREQUENCY (50Hz)
        // ==========================================
        telemetryLoopCounter++;
        
        // Run this block every 5th tick (250Hz / 5 = 50Hz)
        if (telemetryLoopCounter >= 5) {
            
            // These are heavy structs and arrays. 50Hz is plenty fast enough 
            // for debugging wheel slip or PID tuning without bloating your log files.
            driveSpeeds.set(state.Speeds);
            driveModuleStates.set(state.ModuleStates);
            driveModuleTargets.set(state.ModuleTargets);
            driveModulePositions.set(state.ModulePositions);

            /* Telemeterize each module state to a Mechanism2d */
            for (int i = 0; i < 4; ++i) {
                m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
                m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
                m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
            }

            telemetryLoopCounter = 0;
        }


        /* Also write to log file */
        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
        SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");


    }
}
