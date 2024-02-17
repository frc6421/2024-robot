// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class AutoConstants {

    public static final double THETA_P = 1; // TODO update values
    public static final double THETA_I = 0; // TODO update values
    public static final double THETA_D = 0; // TODO update values
    
    public static final double X_DRIVE_P = 2.3; // TODO update values
    public static final double X_DRIVE_I = 0;
    public static final double X_DRIVE_D = 0;

    public static final double Y_DRIVE_P = 2.3; // TODO update values
    public static final double Y_DRIVE_I = 0;
    public static final double Y_DRIVE_D = 0;

    public static final double AUTO_MAX_VELOCITY_METERS_PER_SECOND = 4; // TODO update value
    public static final double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 5; // TODO update value

    public static final double AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 2 * Math.PI;
    public static final double AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC = Math.PI;


  }

  
public static class TrajectoryConstants {
    public static final double CENTER_OF_ROBOT_LENGTH = Units.inchesToMeters(18.625);
    public static final double CENTER_OF_ROBOT_WIDTH = Units.inchesToMeters(17.5);

    public static final Translation2d ORIGIN = new Translation2d(0, 0);

    public static final Translation2d PODIUM = new Translation2d(Units.inchesToMeters(114), Units.inchesToMeters(161.62));

    public static final Translation2d ONE_PIECE_START = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(100)); // THIS IS A COMPLETE GUESS

    public static final Translation2d ONE_PIECE_SHOOT = new Translation2d(Units.inchesToMeters(114), Units.inchesToMeters(100)); // I DONT ACTUALLY KNOW THESE DIMENSIONS (PLS CAD)

    public static final Translation2d TEST_START = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)); // I DONT ACTUALLY KNOW THESE DIMENSIONS (PLS CAD)
    public static final Translation2d TEST_END = new Translation2d(Units.inchesToMeters(72), Units.inchesToMeters(0)); // I DONT ACTUALLY KNOW THESE DIMENSIONS (PLS CAD)

  }

}
