package frc.robot.utilities;

import java.util.HashMap;
import java.util.Map;
//import java.lang.UnsupportedOperationException;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;


/**
 * General purpose logging class
 * Methods always write to WpiLog and will also write to NetworkTables if in debug mode
 * and not on the competition field
 * 
 */

public class RobotLogger {

    private static final String SMARTDASHBOARD_KEY_LABEL = "SmartDashboard";
    private static final String ROOT_KEY_LABEL = "RobotData";

    // 1. Your manual toggle. Set to false before a match, true in the pits.
    public static boolean DEBUG_MODE = true;    
    
    
    // Grab the direct disk logger
    private static DataLog log;
    
    // Caches to hold our log entries so we don't recreate them every 20ms
    private static final Map<String, DoubleLogEntry> doubleLogs = new HashMap<>();
    private static final Map<String, BooleanLogEntry> booleanLogs = new HashMap<>();   
    private static final Map<String, StringLogEntry> stringLogs = new HashMap<>();
    private static final Map<String, StructLogEntry<?>> structLogs = new HashMap<>();
    private static final Map<String, StructPublisher<?>> structPublishers = new HashMap<>();
    private static final Map<String, StructArrayLogEntry<?>> structArrayLogs = new HashMap<>();
    private static final Map<String, StructArrayPublisher<?>> structArrayPublishers = new HashMap<>();

    static {
        // 1. Start the WPILib data logger
        DataLogManager.start();
        
        // 2. Grab the instance
        log = DataLogManager.getLog();
        
        // 3. Start logging Driver Station inputs automatically
        DriverStation.startDataLog(log);
        
        // 4. Print a confirmation message to the console/log
        DataLogManager.log("Custom Logger successfully initialized.");
    }

    private RobotLogger() {
        // from accidentally creating an instance of this class.
        //throw new UnsupportedOperationException("This is a utility class and cannot be instantiated!"); 
        //not working right now 
    }

    private static String getLoggingTableKey() {
        return SMARTDASHBOARD_KEY_LABEL + "/" + ROOT_KEY_LABEL;
    }

    private static String getDecoratedKey(String key) {
        return getRootKey() + key;
    }

    /**
     * Call this single method in robotInit() to force Java to load this class.
     */
    public static void init() {
        // This method is intentionally empty! 
        // Just calling Logger.init() tells Java to load the class, 
        // which automatically triggers the static block above.
    }

    private static boolean inPublishMode() {
        return DEBUG_MODE && !DriverStation.isFMSAttached();
    }
    
    private static String getRootKey() {
        return "/" + ROOT_KEY_LABEL + "/";
    }

    public static void logDouble(String key, double value) {
        DoubleLogEntry entry = doubleLogs.computeIfAbsent(key, 
            k -> new DoubleLogEntry(log, getDecoratedKey(k)));
        entry.append(value);

        if (inPublishMode()) {
            SmartDashboard.putNumber(getDecoratedKey(key), value);
        }
    }

    
    public static void logBoolean(String key, boolean value) {
        BooleanLogEntry entry = booleanLogs.computeIfAbsent(key, 
            k -> new BooleanLogEntry(log, getDecoratedKey(k)));
        entry.append(value);

        if (inPublishMode()) {
            SmartDashboard.putBoolean(getDecoratedKey(key), value);
        }
    }

    
    public static void logString(String key, String value){
        StringLogEntry entry = stringLogs.computeIfAbsent(key, 
            k -> new StringLogEntry(log, getDecoratedKey(k)));
        entry.append(value);

        if(inPublishMode()){
            SmartDashboard.putString(getDecoratedKey(key), value);
        }
    }

    /**
     * Logs a single Struct object (like ChassisSpeeds or Pose2d)
     */
    @SuppressWarnings("unchecked")
    public static <T> void logStruct(String key, edu.wpi.first.util.struct.Struct<T> structType, T value) {
        
        StructLogEntry<T> logEntry = (StructLogEntry<T>) structLogs.computeIfAbsent(key, 
            k -> StructLogEntry.create(log, getDecoratedKey(k), structType));
        logEntry.append(value);

        if (inPublishMode()) {
            StructPublisher<T> publisher = (StructPublisher<T>) structPublishers.computeIfAbsent(key,
                k -> NetworkTableInstance.getDefault().getTable(getLoggingTableKey()).getStructTopic(k, structType).publish());
            publisher.set(value);
        }
    }

    /**
     * Logs an array of Struct objects (like SwerveModuleState[])
     */
    @SuppressWarnings("unchecked")
    public static <T> void logStructArray(String key, edu.wpi.first.util.struct.Struct<T> structType, T[] value) {
        
        StructArrayLogEntry<T> logEntry = (StructArrayLogEntry<T>) structArrayLogs.computeIfAbsent(key, 
            k -> StructArrayLogEntry.create(log, getDecoratedKey(k), structType));
        logEntry.append(value);

        if (inPublishMode()) {
            StructArrayPublisher<T> publisher = (StructArrayPublisher<T>) structArrayPublishers.computeIfAbsent(key,
                k -> NetworkTableInstance.getDefault().getTable(getLoggingTableKey()).getStructArrayTopic(k, structType).publish());
            publisher.set(value);
        }
    }    

// =========================================================
    // SENDABLE LOGGING (putData)
    // =========================================================

    /**
     * Publishes a Sendable (like a PIDController) to NetworkTables for tuning.
     * Automatically shuts off during official matches to save bandwidth.
     * DataLogManager will automatically record the NetworkTable values.
     */
    public static void putDebugData(String key, Sendable data) {
        if (inPublishMode()) {
            SmartDashboard.putData(key, data);
        }
    }

    /**
     * Publishes critical Sendables that the Drive Team MUST interact with 
     * during a real match, such as the Autonomous SendableChooser.
     */
    public static void putMatchData(String key, Sendable data) {
        // No FMS or Debug checks here! Always publish.
        SmartDashboard.putData(key, data);
    }


    /*
    * retrieves the value from a key on SmartDashboard prefixed with
    * the root key where all log data gets written. That way you'll look 
    * in the same location for both logged data and data you're trying to
    * retrieve
    */
    public static double getDouble(String key, double defaultValue) {
        return SmartDashboard.getNumber(getDecoratedKey(key), defaultValue);
    }

}
