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

    public static final double THETA_P = 5; // TODO update values
    public static final double THETA_I = 0; // TODO update values
    public static final double THETA_D = 0; // TODO update values
    
    public static final double X_DRIVE_P = 2.31; // TODO update values
    public static final double X_DRIVE_I = 0;
    public static final double X_DRIVE_D = 0;

    public static final double Y_DRIVE_P = 2.31; // TODO update values
    public static final double Y_DRIVE_I = 0;
    public static final double Y_DRIVE_D = 0;

    public static final double AUTO_MAX_VELOCITY_METERS_PER_SECOND = 4; // TODO update value
    public static final double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 5; // TODO update value

    public static final double AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 2 * Math.PI;
    public static final double AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC = 2 * Math.PI;

    public static final double AUTO_INTAKE_DELAY = 0.4;


  }

  
public static class TrajectoryConstants {

  public static final double CENTER_OF_ROBOT_LENGTH = Units.inchesToMeters(18.375);
  public static final double CENTER_OF_ROBOT_WIDTH = Units.inchesToMeters(17.5);

  public static final Translation2d ORIGIN = new Translation2d(0, 0);

  public static final Translation2d TEST_START = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)); 
  public static final Translation2d TEST_END = new Translation2d(Units.inchesToMeters(72), Units.inchesToMeters(0));

  public static final Translation2d BLUE_SUSSEX_SCORE = new Translation2d(Units.inchesToMeters(128), Units.inchesToMeters(218.5));
  public static final Translation2d RED_SUSSEX_SCORE = new Translation2d(Units.inchesToMeters(523), Units.inchesToMeters(218.5));

  public static final Translation2d NOTE1 = new Translation2d(Units.inchesToMeters(114)-CENTER_OF_ROBOT_LENGTH, Units.inchesToMeters(161.5));
  public static final Translation2d NOTE2 = new Translation2d(Units.inchesToMeters(114)-CENTER_OF_ROBOT_LENGTH, Units.inchesToMeters(218.5));
  public static final Translation2d NOTE3 = new Translation2d(Units.inchesToMeters(114)-CENTER_OF_ROBOT_LENGTH, Units.inchesToMeters(275.5));

  public static final Translation2d NOTE4_BLUE = new Translation2d(Units.inchesToMeters(324)-(CENTER_OF_ROBOT_LENGTH/2), Units.inchesToMeters(30.5));
  public static final Translation2d NOTE5_BLUE = new Translation2d(Units.inchesToMeters(324)-(CENTER_OF_ROBOT_LENGTH/2), Units.inchesToMeters(95.5));
  public static final Translation2d NOTE6_BLUE = new Translation2d(Units.inchesToMeters(324)-(CENTER_OF_ROBOT_LENGTH/2), Units.inchesToMeters(161.5));
  public static final Translation2d NOTE7_BLUE = new Translation2d(Units.inchesToMeters(324)-(CENTER_OF_ROBOT_LENGTH/2), Units.inchesToMeters(227.5));
  public static final Translation2d NOTE8_BLUE = new Translation2d(Units.inchesToMeters(324)-(CENTER_OF_ROBOT_LENGTH/2), Units.inchesToMeters(293.5));

  public static final Translation2d NOTE4_RED = new Translation2d(Units.inchesToMeters(324)+(CENTER_OF_ROBOT_LENGTH/2), Units.inchesToMeters(30.5));
  public static final Translation2d NOTE5_RED = new Translation2d(Units.inchesToMeters(324)+(CENTER_OF_ROBOT_LENGTH/2), Units.inchesToMeters(95.5));
  public static final Translation2d NOTE6_RED = new Translation2d(Units.inchesToMeters(324)+(CENTER_OF_ROBOT_LENGTH/2), Units.inchesToMeters(161.5));
  public static final Translation2d NOTE7_RED = new Translation2d(Units.inchesToMeters(324)+(CENTER_OF_ROBOT_LENGTH/2), Units.inchesToMeters(227.5));
  public static final Translation2d NOTE8_RED = new Translation2d(Units.inchesToMeters(324)+(CENTER_OF_ROBOT_LENGTH/2), Units.inchesToMeters(293.5));

  public static final Translation2d NOTE9 = new Translation2d(Units.inchesToMeters(534)+CENTER_OF_ROBOT_LENGTH, Units.inchesToMeters(161.5));
  public static final Translation2d NOTE10 = new Translation2d(Units.inchesToMeters(534)+CENTER_OF_ROBOT_LENGTH, Units.inchesToMeters(218.5));
  public static final Translation2d NOTE11 = new Translation2d(Units.inchesToMeters(534)+CENTER_OF_ROBOT_LENGTH, Units.inchesToMeters(275.5));

  public static final Translation2d FRONT_CENTER_BLUE_SUBWOOFER = new Translation2d(Units.inchesToMeters(36)+CENTER_OF_ROBOT_LENGTH, Units.inchesToMeters(218.5));
  public static final Translation2d FRONT_CENTER_RED_SUBWOOFER = new Translation2d(Units.inchesToMeters(612)-CENTER_OF_ROBOT_LENGTH, Units.inchesToMeters(218.5));

  public static final Translation2d BLUE_CENTER_LINE_SHOOTING_POSITION = new Translation2d(Units.inchesToMeters(153), Units.inchesToMeters(200));
  public static final Translation2d BLUE_CENTER_LINE_STARTING_POSITION = new Translation2d(Units.inchesToMeters(0)+CENTER_OF_ROBOT_LENGTH, Units.inchesToMeters(80));
  public static final Translation2d BLUE_CENTER_OF_STAGE = new Translation2d(Units.inchesToMeters(190), Units.inchesToMeters(161.5));
  public static final Translation2d BLUE_EDGE_OF_STAGE = new Translation2d(Units.inchesToMeters(220), Units.inchesToMeters(161.5));

  public static final Translation2d RED_CENTER_LINE_SHOOTING_POSITION = new Translation2d(Units.inchesToMeters(495), Units.inchesToMeters(200));
  public static final Translation2d RED_CENTER_LINE_STARTING_POSITION = new Translation2d(Units.inchesToMeters(648)-CENTER_OF_ROBOT_LENGTH, Units.inchesToMeters(80));
  public static final Translation2d RED_CENTER_OF_STAGE = new Translation2d(Units.inchesToMeters(460), Units.inchesToMeters(161.5));
  public static final Translation2d RED_EDGE_OF_STAGE = new Translation2d(Units.inchesToMeters(430), Units.inchesToMeters(161.5));

  //public static final Translation2d RED_FIVE_PIECE_SCORE = new Translation2d(Units.inchesToMeters(470), Units.inchesToMeters(260));
  public static final Translation2d NOTE9_SCORE_POINT = new Translation2d(Units.inchesToMeters(567), Units.inchesToMeters(200));
  public static final Translation2d RED_DONT_HIT_WALL = new Translation2d(Units.inchesToMeters(495), Units.inchesToMeters(300));

  //public static final Translation2d BLUE_FIVE_PIECE_SCORE = new Translation2d(Units.inchesToMeters(178), Units.inchesToMeters(260));
  public static final Translation2d NOTE1_SCORE_POINT = new Translation2d(Units.inchesToMeters(81), Units.inchesToMeters(200));
  public static final Translation2d BLUE_DONT_HIT_WALL = new Translation2d(Units.inchesToMeters(153), Units.inchesToMeters(300));

  public static final Translation2d BLUE_FOUR_PIECE_START = new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(266));
  public static final Translation2d RED_FOUR_PIECE_START = new Translation2d(Units.inchesToMeters(618), Units.inchesToMeters(266));

  public static final Translation2d RED_CENTER_SCORE = new Translation2d(Units.inchesToMeters(503), Units.inchesToMeters(260));
  public static final Translation2d BLUE_CENTER_SCORE = new Translation2d(Units.inchesToMeters(145), Units.inchesToMeters(260));

  }

  public static enum RobotStates{
    DRIVE,
    INTAKE,
    SPEAKER,
    AMP,
    TRAP,
    CLIMB,
    BARF,
    SHUTTLE
  }

  public static enum ClimberStates {
    PREPARE_CLIMB, // Raise both arms up to middle position, drive backwards.
    ARMS_HIGH, // Raise both arms to the climbing position
    CLIMB // Lower climber arms
    // PREPARE_TRAP, // Raise transition arm to trap degree
    // SCORE_TRAP // Spit out note
  }
    
  public static class VisionConstants {

    //TODO change these to degrees recorded in notebook
    /** In degrees */
    public static final double AMP_CLOSE_PITCH_ANGLE = 1.80;

    /** In degrees */
    public static final double AMP_CLOSE_YAW_ANGLE = -1.26;

    /** In degrees */
    public static final double AMP_FAR_PITCH_ANGLE = 17.99;

    /** In degrees */
    public static final double AMP_FAR_YAW_ANGLE = 0.95;

    /** In degrees */
    // Only first and last values are used to check edge cases with equation
    public static final double[] SPEAKER_PITCH_ARRAY = {16.71, 9.21, 3.83, -0.51, -3.90, -6.56, -8.68, -10.56, -12.12, -13.37, -14.49, -15.59, -16.39};

    /** In degrees */
    public static final double[] SPEAKER_YAW_ARRAY = {0.0};

    public static final double SPEAKER_YAW_ANGLE = 4.24;

    //TODO update both arrays
    /** In degrees */
    public static final double[] SHOOTER_PIVOT_ARRAY = {52, 50, 45.5, 42, 38.25, 35.25, 32.75, 31, 29.75, 28, 26, 25.55, 24.4};

    /** In RPM */
    public static final double[] SHOOTER_RPM_ARRAY = {3500, 3600, 3700, 3800, 3900, 4000, 4100, 4250, 4400, 4550, 4700, 4900, 5100};

    /** In degrees */
    public static final double TRAP_PITCH_ANGLE = -16.0;

    /** In degrees */
    public static final double TRAP_YAW_ANGLE = 10.5;

  }

}
