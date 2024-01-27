// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransitionSubsystem extends SubsystemBase {
  
  private final CANSparkFlex transitionMotor;
  public static class TransitionConstants {
    
    private static final int TRANSITION_MOTOR_CAN_ID = 21;
    //TODO Find the Channels
    private static final int TOP_PROXIMITY_SENSOR_DIO = 0;  
    private static final int BOTTOM_PROXIMITY_SENSOR_DIO = 0; 
  }
  private final DigitalInput topProximitySensor;
  private final DigitalInput bottomProximitySensor;
  /** Creates a new TransitionSubsystem. */
  public TransitionSubsystem() {
    // Make new instance of motor
    transitionMotor = new CANSparkFlex(TransitionConstants.TRANSITION_MOTOR_CAN_ID, MotorType.kBrushless);
  
    topProximitySensor = new DigitalInput(TransitionConstants.TOP_PROXIMITY_SENSOR_DIO);
    bottomProximitySensor = new DigitalInput(TransitionConstants.BOTTOM_PROXIMITY_SENSOR_DIO);
    
    // Factory default and inversion
    transitionMotor.restoreFactoryDefaults();
    //TODO Verify Inversion
    transitionMotor.setInverted(false);
    // Set to idle to coast
    transitionMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);

  }
  /** Sets the belts to the Intake mode
   * 
   * @param value Used to set the speed of the belt
  */
  public void SetTransitionIntakeState(double value) {
    transitionMotor.set(value);
  }

  /**
   * Comparing topProximitySenson and bottomProximitySensor boolean values
   * @return true/false
  */
  public boolean isNoteInTransition() {
    if(topProximitySensor.get() == true && bottomProximitySensor.get() == true) {
      return true;
    }
    else
    {
      return false;
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

