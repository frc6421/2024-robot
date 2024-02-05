// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransitionSubsystem extends SubsystemBase {

  public static class TransitionConstants {

    private static final int TRANSITION_MOTOR_CAN_ID = 21;
    private static final int TIME_OF_FLIGHT_SENSOR_IN_CAN_ID = 22;
    private static final int TIME_OF_FLIGHT_SENSOR_OUT_CAN_ID = 23;

    private static final double TRANSITION_GEAR_RATIO = 1.6;

    // TOF Sensors
    // TODO confirm this value
    private static final double DETECTION_DISTANCE_MM = 100;

    public static final double TRANSITION_FORWARD_SPEED = 0.85;
    public static final double TRANSITION_REVERSE_SPEED = -0.85;
    
  }

  private final CANSparkFlex transitionMotor;

  private final RelativeEncoder transitionEncoder;
  public final TimeOfFlight timeOfFlightIn;
  public final TimeOfFlight timeOfFlightOut;

  /** Creates a new TransitionSubsystem. */
  public TransitionSubsystem() {
    // Make new instance of motor
    transitionMotor = new CANSparkFlex(TransitionConstants.TRANSITION_MOTOR_CAN_ID, MotorType.kBrushless);

    transitionEncoder = transitionMotor.getEncoder(); 

    timeOfFlightIn = new TimeOfFlight(TransitionConstants.TIME_OF_FLIGHT_SENSOR_IN_CAN_ID);
    timeOfFlightOut = new TimeOfFlight(TransitionConstants.TIME_OF_FLIGHT_SENSOR_OUT_CAN_ID);

    // Factory default and inversion
    transitionMotor.restoreFactoryDefaults();

    transitionMotor.setInverted(true);

    // Set to idle to coast
    transitionMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);

    // Gear Ratio
    transitionEncoder.setPositionConversionFactor(TransitionConstants.TRANSITION_GEAR_RATIO);

    //Shuffleboard
    Shuffleboard.getTab("Trasition").add(this);
  }

  /** Sets the belts to the Intake mode
   * 
   * @param value Used to set the output of the belts
  */
  public void setTransitionMotorSpeed(double value) {
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
    double output = transitionMotor.get();
    return output;
  }

  /** Comparing topProximitySensor and bottomProximitySensor boolean values
   * @return true when both sensors are detecting a note, else false
  */
  // public boolean isNoteInTransition() {
  //   // Compares values in a boolean statement
  //   return (topProximitySensor.get() && bottomProximitySensor.get());
  // }

  /** Takes in the values of timeOfFlight1 and timeOfFlight2 and compares them
   *  @return true/false (true = close to centered, false = not close to center)
   */
  // TODO make a command to move the belts when it is not centered
  // TODO test if position in transition matters when shooting

  public boolean isCentered() {
    if (timeOfFlightOut.getRange() < TransitionConstants.DETECTION_DISTANCE_MM) {
      return true;
    }
    else {
      return false;
    }
  }

  public double getTOFInRange() {
    return timeOfFlightIn.getRange();
  }

  public double getTOFOutRange() {
    return timeOfFlightOut.getRange();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

   public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("SwerveModule");

    //builder.addBooleanProperty("Transition Sensor Output", this::isNoteInTransition, null);
    builder.addDoubleProperty("Transition Motor Output", this::getTransitionMotorOutput, null);
    builder.addBooleanProperty("Time Of Flight Boolean Output", this::isCentered, null);
    builder.addDoubleProperty("Time Of Flight 1 Output", this::getTOFInRange, null);
    builder.addDoubleProperty("Time Of Flight 2 Output", this::getTOFOutRange, null);
  }
}

