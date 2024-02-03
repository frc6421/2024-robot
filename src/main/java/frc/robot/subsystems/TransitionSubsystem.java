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
    private static final int TIME_OF_FLIGHT_SENSOR_1_CAN_ID = 22;
    private static final int TIME_OF_FLIGHT_SENSOR_2_CAN_ID = 23;
    //TODO Find the Channels
    private static final int TOP_PROXIMITY_SENSOR_DIO = 0;  
    private static final int BOTTOM_PROXIMITY_SENSOR_DIO = 0;

    private static final double TRANSITION_GEAR_RATIO = 1.6;

    // TOF Sensors
    // TODO confirm this value
    private static final double DETECTION_DISTANCE_MM = 100;
    
    public static enum transitionState {
      INTAKE,
      SHOOTING,
      AMP,
      // TODO Do we need amp and trap to be seperate states?
      TRAP
    }
  }

  private final CANSparkFlex transitionMotor;

  private final DigitalInput topProximitySensor;
  private final DigitalInput bottomProximitySensor;

  private final RelativeEncoder transitionEncoder;
  public final TimeOfFlight timeOfFlightTop;
  public final TimeOfFlight timeOfFlightBottom;
  /** Creates a new TransitionSubsystem. */
  public TransitionSubsystem() {
    // Make new instance of motor
    transitionMotor = new CANSparkFlex(TransitionConstants.TRANSITION_MOTOR_CAN_ID, MotorType.kBrushless);
  
    // Make 2 new instances of DigitalInput
    topProximitySensor = new DigitalInput(TransitionConstants.TOP_PROXIMITY_SENSOR_DIO);
    bottomProximitySensor = new DigitalInput(TransitionConstants.BOTTOM_PROXIMITY_SENSOR_DIO);

    transitionEncoder = transitionMotor.getEncoder(); 

    timeOfFlightTop = new TimeOfFlight(TransitionConstants.TIME_OF_FLIGHT_SENSOR_1_CAN_ID);
    timeOfFlightBottom = new TimeOfFlight(TransitionConstants.TIME_OF_FLIGHT_SENSOR_1_CAN_ID);

    // Factory default and inversion
    transitionMotor.restoreFactoryDefaults();

    //TODO Verify Inversion
    transitionMotor.setInverted(false);

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
  public void setTransitionSpeed(double value) {
    transitionMotor.set(value);
  }

  /** get the output of the transition motor, -1.0 to 1.0
   * @return output
   */
  public double getTransitionOutput() {
    double output = transitionMotor.get();
    return output;
  }

  /** Comparing topProximitySensor and bottomProximitySensor boolean values
   * @return true when both sensors are detecting a note, else false
  */
  public boolean isNoteInTransition() {
    // Compares values in a boolean statement
    return (topProximitySensor.get() && bottomProximitySensor.get());
  }

  /** Takes in the values of timeOfFlight1 and timeOfFlight2 and compares them
   *  @return true/false (true = close to centered, false = not close to center)
   */
  // TODO make a command to move the belts when it is not centered
  // TODO test if position in transition matters when shooting

  public boolean isCentered() {
    if (timeOfFlightTop.getRange() < TransitionConstants.DETECTION_DISTANCE_MM) {
      return true;
    }
    else {
      return false;
    }
  }
  public double getFOTTopRange() {
    return timeOfFlightTop.getRange();
  }

  public double getFOTBottomRange() {
    return timeOfFlightBottom.getRange();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

   public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("SwerveModule");

    builder.addBooleanProperty("Transition Sensor Output", this::isNoteInTransition, null);
    builder.addDoubleProperty("Transition Motor Output", this::getTransitionOutput, null);
    builder.addBooleanProperty("Time Of Flight Boolean Output", this::isCentered, null);
    builder.addDoubleProperty("Time Of Flight 1 Output", this::getFOTTopRange, null);
    builder.addDoubleProperty("Time Of Flight 2 Output", this::getFOTBottomRange, null);
  }
}

