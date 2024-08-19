// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
  public static final Translation2d RED_DONT_HIT_WALL = new Translation2d(Units.inchesToMeters(528), Units.inchesToMeters(290));

  //public static final Translation2d BLUE_FIVE_PIECE_SCORE = new Translation2d(Units.inchesToMeters(178), Units.inchesToMeters(260));
  public static final Translation2d NOTE1_SCORE_POINT = new Translation2d(Units.inchesToMeters(81), Units.inchesToMeters(200));
  public static final Translation2d BLUE_DONT_HIT_WALL = new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(290));

  public static final Translation2d BLUE_FOUR_PIECE_START = new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(266));
  public static final Translation2d RED_FOUR_PIECE_START = new Translation2d(Units.inchesToMeters(618), Units.inchesToMeters(266));

  public static final Translation2d RED_CENTER_SCORE = new Translation2d(Units.inchesToMeters(503), Units.inchesToMeters(260));
  public static final Translation2d BLUE_CENTER_SCORE = new Translation2d(Units.inchesToMeters(145), Units.inchesToMeters(260));


  // amp auto constants slay slay slay
  public static final Translation2d BLUE_AMP_START = new Translation2d(Units.inchesToMeters(0)+CENTER_OF_ROBOT_LENGTH, Units.inchesToMeters(276));
  public static final Translation2d RED_AMP_START = new Translation2d(Units.inchesToMeters(648)-CENTER_OF_ROBOT_LENGTH, Units.inchesToMeters(276));

  public static final Translation2d BLUE_AMP = new Translation2d(Units.inchesToMeters(73), Units.inchesToMeters(325)-CENTER_OF_ROBOT_LENGTH);
  public static final Translation2d RED_AMP = new Translation2d(Units.inchesToMeters(579), Units.inchesToMeters(325)-CENTER_OF_ROBOT_LENGTH);





  public static final double RPM_AT_CENTER_SCORE = 4400;
  public static final double RPM_AT_CENTER_LINE_SHOOTING_POSITION = 4250; // MROC was 4300
  public static final double RPM_AT_FOUR_PIECE_SCORE = 3650;
  public static final double RPM_AT_SUBWOOFER = 3500;

  public static final double DEGREE_AT_CENTER_SCORE = 28.75;
  public static final double DEGREE_AT_CENTER_LINE_SHOOTING_POSITION = 29.5;
  public static final double DEGREE_AT_FOUR_PIECE_SCORE = 45.5;
  public static final double DEGREE_AT_SUBWOOFER = 52;

  }

  public static enum RobotStates{
    DRIVE,
    INTAKE,
    SPEAKER,
    AMP,
    TRAP,
    CLIMB,
    BARF,
    SHUTTLE,
    ONE
  }

  public static enum ClimberStates {
    PREPARE_CLIMB, // Raise both arms up to middle position, drive backwards.
    //ARMS_HIGH // Raise both arms to the climbing position
    CLIMB // Lower climber arms
    // PREPARE_TRAP, // Raise transition arm to trap degree
    // SCORE_TRAP // Spit out note
  }
    
  public static class VisionConstants {

    // Camera Offsets from Center of Robot

    /** In Meters */
    public static final double SPEAKER_CAMERA_X = -0.37; //Units.inchesToMeters(0); // -13.949
    public static final double SPEAKER_CAMERA_Y = Units.inchesToMeters(-1.75); // -1.75
    public static final double SPEAKER_CAMERA_Z = 0.41; //Units.inchesToMeters(0); // 14.05

    /** In Radians */
    public static final double SPEAKER_CAMERA_PITCH = Units.degreesToRadians(-29); //29
    public static final double SPEAKER_CAMERA_YAW = Units.degreesToRadians(180);


    /** In Meters */
    public static final double AMP_CAMERA_X = -0.36; // -14.237
    public static final double AMP_CAMERA_Y = Units.inchesToMeters(1.75); // 1.75
    public static final double AMP_CAMERA_Z = 0.39; // 14.995

    /** In Radians */
    public static final double AMP_CAMERA_PITCH = Units.degreesToRadians(-62); //62
    public static final double AMP_CAMERA_YAW = Units.degreesToRadians(180);

    /** In degrees */
    public static final double AMP_CLOSE_PITCH_ANGLE = 1.27;

    /** In degrees */
    public static final double AMP_CLOSE_YAW_ANGLE = -1.99;

    /** In degrees */
    public static final double AMP_FAR_PITCH_ANGLE = 18.56;

    /** In degrees */
    public static final double AMP_FAR_YAW_ANGLE = 2.40;

    /** In degrees */
    // Only first and last values are used to check edge cases with equation
    public static final double[] SPEAKER_PITCH_ARRAY = {16.71, 9.21, 3.83, -0.51, -3.90, -6.56, -8.68, -10.56, -12.12, -13.37, -14.49, -15.59, -16.39};

    /** In degrees */
    public static final double[] SPEAKER_YAW_ARRAY = {0.0};

    public static final double SPEAKER_YAW_ANGLE = 1.32; //4.24

    //TODO update both arrays
    /** In degrees */
    public static final double[] SHOOTER_PIVOT_ARRAY = {52, 50, 45.5, 42, 38.25, 35.25, 32.75, 31, 29.75, 28, 26, 25.55, 24.4};

    /** In RPM */
    public static final double[] SHOOTER_RPM_ARRAY = {3500, 3600, 3700, 3800, 3900, 4000, 4100, 4250, 4400, 4550, 4700, 4900, 5100};

    /** In degrees */
    public static final double TRAP_PITCH_ANGLE = -16.0;

    /** In degrees */
    public static final double TRAP_YAW_ANGLE = 10.5;


    //TODO add values to all values below this
    /** In poses */
    public static final double MAXIMUM_X_POSE = 0;
    public static final double MAXIMUM_Y_POSE = 0;
    public static final double MAXIMUM_Z_POSE = 0;

    /** In Meters */
    public static final double APRILTAG_METERS_LIMIT = 0;

    /**No Metric */
    public static final double STARTING_STANDARD_DEVIATION = 0;
    public static final double TAG_DISTANCE_WEIGHT = 0;
    public static final double MAXIMUM_AMBIGUITY = 0;

    public static final  Matrix<N3, N1> SD_HIGH_CONFIDENCE = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));
    public static final  Matrix<N3, N1> SD_LOW_CONFIDENCE = VecBuilder.fill(0.9, 0.9, Units.degreesToRadians(10));
  }

}
