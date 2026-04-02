/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants { 
  public static final class Simulation {
    public static final Pose2d LEFT_BOTTOM_CORNER = new Pose2d(new Translation2d(-8.2423, -4.0513), new Rotation2d());  // 2023 game LEFT_BOTTOM_CORNER was at -8.2423, -4.0513
    public static final int simAlliance = 1; // 1 for Red Alliance and -1 for Blue Alliance
  }

  public static final class FieldLocations{
    //Shooting   
    public static final Translation2d BLUE_HUB = new Translation2d(Units.inchesToMeters(182), Units.inchesToMeters(158.5));
    public static final Translation2d RED_HUB = new Translation2d(Units.inchesToMeters(469), Units.inchesToMeters(158.5));

    //Ferrying
    public static final Translation2d BLUE_LEFT_FERRY_POINT = new Translation2d(Units.inchesToMeters(91.055), Units.inchesToMeters(79.4225));
    public static final Translation2d BLUE_RIGHT_FERRY_POINT = new Translation2d(Units.inchesToMeters(91.055), Units.inchesToMeters(238.2675));
    public static final Translation2d RED_LEFT_FERRY_POINT = new Translation2d(Units.inchesToMeters(560.165), Units.inchesToMeters(238.2675));
    public static final Translation2d RED_RIGHT_FERRY_POINT = new Translation2d(Units.inchesToMeters(560.165), Units.inchesToMeters(79.4225));
  }

  public static final class ShooterConstants{
    public static final double BOT_TO_SHOOTER_DISTANCE = -Math.sqrt(Math.pow(Units.inchesToMeters(5.75), 2) + Math.pow(Units.inchesToMeters(-6.75), 2)); //shouldn't be negative
    public static final double BOT_TO_SHOOTER_ANGLE = Math.atan2(Units.inchesToMeters(5.75), Units.inchesToMeters(-6.75));

    public static final Transform2d  BOT_TO_SHOOTER_TRANSFORM = new Transform2d(-6.75, 5.75, new Rotation2d());
  }

  public static final class TurretConstants {
    public static final int TURRET_ID = 30;

    public static final double OFFSET = 0.0;  

    public static final double MAX_ANGLE = 248.2 - 20.0 + (OFFSET * 10.0/360.0);
    public static final double MIN_ANGLE = -149.988 + 20.0 + (OFFSET * 10.0/360.0);
  }

  public static final class FlywheelConstants {
    public static final int MOTOR_ID = 17;

    public static final double S = 0.55;
    public static final double V = 0.12167;
    public static final double P = 6.0;
    public static final double I = 0.0;
    public static final double D = 0.0;
  }

  public static final class HoodConstants {
    public static final int MOTOR_ID = 18;
    public static final int ENCODER_ID = 20;

    public static final double ENCODER_MAGNET_OFFSET = 0.716064;
    public static final double ENCODER_MIN = 0.0;
    public static final double ENCODER_MAX = -0.831299;

    public static final double P = 7.0;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final double GEAR_RATIO = 350.0/25.0;

    public static final double MIN_ANGLE = ENCODER_MIN/GEAR_RATIO * 360;
    public static final double MAX_ANGLE = ENCODER_MAX/GEAR_RATIO * 360;
  }

  public static final class IntakeConstants {
    public static final int ROLLER_ID = 14;
    public static final int PIVOT_ID = 13;
    public static final int ENCODER_ID = 19;

    //new values with encoder
    public static final double INTAKE_RETRACTED = 0.419434; //check these later
    public static final double INTAKE_DEPLOYED = 0.247314; //check these later
  }

  public static final class FeederConstants {
    public static final int FEEDER_ID = 16;

    public static final double FEEDER_SPEED = 10.0; //check this later
  }

  public static final class SpindexerConstants {
    public static final int SPINDEXER_ID = 15;

    public static final double SPINDEXER_SPEED = 12.0; //check this later
  }

  public static final class KickerConstants {
    public static final int KICKER_ID = 22;

    public static final double KICKER_SPEED = 6.0;
  }

  public static final class ClimberConstants {
    public static final int CLIMBER_ID = 19;
  }

  public static final class LimelightConstants {
    public static final double RIGHT_LL_FORWARD_OFFSET_METERS = Units.inchesToMeters(10.5); // + is forward from robot center
    public static final double RIGHT_LL_LEFT_OFFSET_METERS = Units.inchesToMeters(10.5); // + is left, - is right
    public static final double RIGHT_LL_UP_OFFSET_METERS = Units.inchesToMeters(10.5); // + is up from the floor
    public static final double RIGHT_LL_ROLL_OFFSET_DEGREES = 0.0; // Tilted left/right
    public static final double RIGHT_LL_PITCH_OFFSET_DEGREES = 0.0; // Tilted up/down (+ is looking up)
    public static final double RIGHT_LL_YAW_OFFSET_DEGREES =  127.927226; // Turned left/right (+ is turned left)               

    public static final double LEFT_LL_FORWARD_OFFSET_METERS = Units.inchesToMeters(10.5);
    public static final double LEFT_LL_LEFT_OFFSET_METERS = Units.inchesToMeters(10.5);
    public static final double LEFT_LL_UP_OFFSET_METERS = Units.inchesToMeters(10.5);
    public static final double LEFT_LL_ROLL_OFFSET_DEGREES = 0.0;
    public static final double LEFT_LL_PITCH_OFFSET_DEGREES = 0.0;
    public static final double LEFT_LL_YAW_OFFSET_DEGREES = -50.357;                
  }

}
// ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠿⢿⠿⠷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⠛⠉⠀⠀⠀⠐⠒⠒⠒⠺⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠏⡰⠃⠄⠀⠀⠀⠀⠆⠀⠀⢢⠈⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠏⣐⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠈⠂⠙⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠉⢉⡄⢱⣿⣿⣿⣾⣿⣿⣷⣦⡀⠀⢀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠀⢸⡇⠸⣿⣿⣿⣿⣿⣿⣿⣿⣧⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣏⠂⣀⡜⠁⠀⠂⠀⢹⣯⠉⠉⠛⠻⠏⠀⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡏⠀⢻⡇⢸⣿⣷⣆⣸⣿⣷⣶⣶⣿⠀⠼⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⠀⢸⡇⣸⣿⠟⣀⣠⣈⣿⣻⣿⡿⠀⢰⣭⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠶⠾⠁⣿⣥⣅⣉⣛⣛⠛⣩⠗⢁⣀⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡃⠠⡌⠻⣿⣿⣷⣿⣿⡏⢰⣿⣿⣿⣟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠏⠀⣦⣍⣀⣈⣉⣡⣊⡅⠈⢴⣶⣭⣛⣿⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⢃⡄⡄⢿⣿⣿⣿⣿⣿⡿⢁⡇⠀⠙⠿⣿⣷⡽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⡿⠿⠘⢰⣿⡇⠓⠚⠻⢿⣿⠿⢋⣴⣿⠁⠀⠀⠀⠀⠀⠈⠑⠛⠛⠻⠿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⡿⢟⡫⠉⠉⠀⠀⠀⣿⣿⣼⣿⣯⠁⠘⢁⣴⢿⣿⠟⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⠋⠁⠒⠉⠀⠀⠀⠀⠀⢳⣿⠿⡟⣿⣿⠘⢰⣷⣿⣿⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠰⡇⣿⣿⠠⠤⢬⣉⣉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⣿⠃⠘⠒⠲⠤⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢰⣿⡷⢠⢉⣈⡓⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⡏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣺⣿⡇⠤⢤⣭⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣿⣿⣿⣿⣿⣿⣿⣿
// ⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⣿⡇⠒⠶⠖⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠻⢿⣿⣿⣿⣿⣿
// ⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⣿⢉⣉⣉⠁⠀⠀⠀⠀⠐⠒⠶⣶⣶⣶⣤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⠿⣿⣿⣿
// ⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢼⣿⠠⠤⣌⠀⠀⠀⠀⠀⠰⠉⣴⣶⣶⣿⣿⣿⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠻⣿
// ⣷⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠺⣸⠐⠲⠄⠀⠀⠀⠀⠀⠀⠈⣿⠛⠛⣛⣿⣿⣿⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿
// ⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣹⣿⠈⠒⠂⠀⠀⠀⠀⠀⠀⠀⠘⠚⠛⠻⠿⣿⣿⣷⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢿
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢼⣽⠀⢉⡁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⠿⠏⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸
// ⠄⠀⠀⠀⣠⣤⠶⠿⢿⣻⣄⠀⠀⡼⣿⠀⠤⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸
// ⣾⠀⠀⠜⠁⢼⣶⣯⣤⣤⡴⠀⠀⢹⡟⠐⠒⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⡀⠀⠀⠀⣀⣀⣀⣀⣴⣿⣿
// ⣿⡄⠀⠀⠀⠀⢿⣦⣤⠤⠒⠀⠀⣸⡏⠀⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣶⣄⡀⠀⠘⠒⠖⠒⠀⠀⠀⢹⣷⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣦⣄⣀⡀⠀⠀⠀⠀⢸⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⣿⠁⠀⠀⠀⣤⣼⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
// ⣿⣿⣿⣿⣿⣿⣿⡟⠀⠀⠀⠀⣿⣿⢿⣷⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
