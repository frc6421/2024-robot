// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransitionSubsystem extends SubsystemBase {

  public static class TransitionConstants {

    private static final int TRANSITION_MOTOR_CAN_ID = 21;
    //TODO Find the Channels
    private static final int TOP_PROXIMITY_SENSOR_DIO = 0;  
    private static final int BOTTOM_PROXIMITY_SENSOR_DIO = 0;

    private static final double TRANSITION_GEAR_RATIO = 1.6;
    
    //TODO Velocity???
  }

  private final CANSparkFlex transitionMotor;

  private final DigitalInput topProximitySensor;
  private final DigitalInput bottomProximitySensor;

  /** Creates a new TransitionSubsystem. */
  public TransitionSubsystem() {
    // Make new instance of motor
    transitionMotor = new CANSparkFlex(TransitionConstants.TRANSITION_MOTOR_CAN_ID, MotorType.kBrushless);
  
    // Make 2 new instances of DigitalInput
    topProximitySensor = new DigitalInput(TransitionConstants.TOP_PROXIMITY_SENSOR_DIO);
    bottomProximitySensor = new DigitalInput(TransitionConstants.BOTTOM_PROXIMITY_SENSOR_DIO);
    
    // Factory default and inversion
    transitionMotor.restoreFactoryDefaults();
    //TODO Verify Inversion
    transitionMotor.setInverted(false);
    // Set to idle to coast
    transitionMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);

    //Shuffleboard
    Shuffleboard.getTab("Trasition").add(this);
  }
  /** Sets the belts to the Intake mode
   * 
   * @param value Used to set the output of the belts
  */
  public void setTransitionIntakeState(double value) {
    transitionMotor.set(value);
  }
  /** get the output of the transition motor, -1.0 to 1.0
   * @return output
   */
  public double getTransitionOutput() {
    double output = transitionMotor.get();
    return output;
  }

  /**
   * Comparing topProximitySensor and bottomProximitySensor boolean values
   * @return true when both sensors are detecting a note, else false
  */
  public boolean isNoteInTransition() {
    // Compares values in a boolean statement
    return (topProximitySensor.get() && bottomProximitySensor.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

   public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("SwerveModule");

    builder.addBooleanProperty("Transition Motor Output", this::isNoteInTransition, null);
    builder.addDoubleProperty("Transition Motor Output", this::getTransitionOutput, null);
  }
}

