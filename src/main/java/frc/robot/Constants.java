/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants { 
  public static final class Simulation {
    public static final Pose2d LEFT_BOTTOM_CORNER = new Pose2d(new Translation2d(-8.2423, -4.0513), new Rotation2d());  // 2023 game LEFT_BOTTOM_CORNER was at -8.2423, -4.0513
    public static final int simAlliance = 1; // 1 for Red Alliance and -1 for Blue Alliance
  }

}
