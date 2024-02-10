// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransitionSubsystem extends SubsystemBase {

  public static class TransitionConstants {

    private static final int TRANSITION_MOTOR_CAN_ID = 21;
    private static final int TIME_OF_FLIGHT_SENSOR_IN_CAN_ID = 25;
    private static final int TIME_OF_FLIGHT_SENSOR_OUT_CAN_ID = 24;

    private static final double TRANSITION_GEAR_RATIO = 1.6;

    public static final double TRANSITION_FORWARD_SPEED = 0.85;
    public static final double TRANSITION_REVERSE_SPEED =  -0.85;

    // TODO confirm this value
    public static final double DETECTION_DISTANCE_MM = 100;

    //PID
    private static final double TRANSITION_KP = 0;
    private static final double TRANSITION_KS = 0;
  }



  private final CANSparkFlex transitionMotor;

  private final RelativeEncoder transitionEncoder;
  private final SparkPIDController transitionPIDController;
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

    // Set PID Controller
    transitionPIDController = transitionMotor.getPIDController();

    // Set Encoder
    transitionEncoder = transitionMotor.getEncoder(); 

    // Inversion
    transitionMotor.setInverted(true);

    // Set P
    transitionPIDController.setP(TransitionConstants.TRANSITION_KP);

    // Set to idle to coast
    transitionMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);

    // Gear Ratio
    transitionEncoder.setPositionConversionFactor(TransitionConstants.TRANSITION_GEAR_RATIO);

    //Shuffleboard
    Shuffleboard.getTab("Transition").add(this);
    Shuffleboard.getTab("Transition").add(timeOfFlightIn);
    Shuffleboard.getTab("Transition").add(timeOfFlightOut);
  }

  /** Sets the belts to a given output
   *  value (-1.0 - 1.0)
   * @param value Used to set the output of the belts
  */
  public void setTransitionMotorOutput(double value) {
    transitionMotor.set(value);
  }

  /**
   * Stops the transition motor
   * 
   */
  public void stopTransitionMotor() {
    transitionMotor.stopMotor();
  }

  /** get the output of the transition motor, -1.0 to 1.0
   * @return output
   */
  public double getTransitionMotorOutput() {
   return transitionMotor.get();
  }

  /** Takes in the values of timeOfFlightIn
   *  @return true/false (true = in the transition, false = not in the transitions)
   */
  public boolean isNoteDetected() {
    if (timeOfFlightIn.getRange() < TransitionConstants.DETECTION_DISTANCE_MM) {
      return true;
    }
    else {
      return false;
    }
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

  /**Sets the given velocity to the transitionMotor
   * @param double velocity
   */
  public void setTransitionVoltage(double velocity) {
    transitionMotor.setVoltage(velocity);
  }

  /** Sets the position of the PID controller
   * @param double position (the position you want to set it to)
   */
  public void setTransitionPosition(double position) {
    transitionPIDController.setReference(position, CANSparkBase.ControlType.kPosition, 0, TransitionConstants.TRANSITION_KS, ArbFFUnits.kVoltage);
  }

  /** Sets the PID controller P value
   * @param double P (the P value)
   */
  public void setTransitionMotorP(double P) {
    transitionPIDController.setP(P);
  }

  /** Gets the current position of the encoder
   * Unit - Rotations
   * @return Position in rotations
   */
  public double getPosition() {
    return transitionEncoder.getPosition();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

   public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("SwerveModule");

    //builder.addBooleanProperty("Transition Sensor Output", this::isNoteInTransition, null);
    builder.addDoubleProperty("Transition Motor Output", this::getTransitionMotorOutput, null);
    builder.addDoubleProperty("Time Of Flight In Output", this::getTOFInRange, null);
    builder.addDoubleProperty("Time Of Flight Out Output", this::getTOFOutRange, null);
   }
}

