// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransitionSubsystem extends SubsystemBase {

  public static class TransitionConstants {

    private static final int TRANSITION_MOTOR_CAN_ID = 21;
    private static final int TIME_OF_FLIGHT_SENSOR_IN_CAN_ID = 25;
    private static final int TIME_OF_FLIGHT_SENSOR_OUT_CAN_ID = 24;

    private static final double TRANSITION_GEAR_RATIO = 1.6;

    public static final double TRANSITION_SPEED = 0.3 * 12;
    public static final double AMP_TRANSITION_SPEED = 0.5 * 12;

    // Its about 400-380 when nothing is detected
    public static final double DETECTION_DISTANCE_MM = 350; // 350
  }

  private final CANSparkFlex transitionMotor;

  private final RelativeEncoder transitionEncoder;
  public final TimeOfFlight timeOfFlightIn;
  public final TimeOfFlight timeOfFlightOut;

  /** Creates a new TransitionSubsystem. */
  public TransitionSubsystem() {
    // Make new instance of motor
    transitionMotor = new CANSparkFlex(TransitionConstants.TRANSITION_MOTOR_CAN_ID, MotorType.kBrushless);

    // Factory default
    transitionMotor.restoreFactoryDefaults();

    // Set TOF
    timeOfFlightIn = new TimeOfFlight(TransitionConstants.TIME_OF_FLIGHT_SENSOR_IN_CAN_ID);
    timeOfFlightOut = new TimeOfFlight(TransitionConstants.TIME_OF_FLIGHT_SENSOR_OUT_CAN_ID);

    // Set Encoder
    transitionEncoder = transitionMotor.getEncoder(); 

    // Inversion
    transitionMotor.setInverted(true);

    // Set to idle to coast
    transitionMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);

    // Gear Ratio
    transitionEncoder.setPositionConversionFactor(TransitionConstants.TRANSITION_GEAR_RATIO);
  }

  /**Returns the double value of the TOF In sensor
   * Unit - MM
   * @return double
   */
  public double getTOFInRange() {
    return timeOfFlightIn.getRange();
  }
  /**Returns the double value of the TOF Out sensor
   * Unit - MM
   * @return double
   */
  public double getTOFOutRange() {
    return timeOfFlightOut.getRange();
  }

  /**Sets the given voltage to the transitionMotor
   * @param double voltage
   */
  public void setTransitionVoltage(double voltage)
  {
    transitionMotor.setVoltage(voltage);
  }

  /**
   * Stops the transition motors
   */
  public void stopTransition() {
    transitionMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

