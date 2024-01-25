// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransitionSubsystem extends SubsystemBase {
  
  private final CANSparkMax transitionMotor;
  public static class TransitionConstants {
    private static final int TRANSITION_MOTOR_CAN_ID = 21;
  }
  /** Creates a new TransitionSubsystem. */
  public TransitionSubsystem() {
    // Make new instance of motor
    transitionMotor = new CANSparkMax(TransitionConstants.TRANSITION_MOTOR_CAN_ID, MotorType.kBrushless);
  
    // Factory default
    transitionMotor.restoreFactoryDefaults();

    // Set to idle to coast
    transitionMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }
  /** Sets the belts to the Intake mode
   * 
   * @param value Used to set the speed of the belt
  */
  public void SetIntakeState(double value) {
    transitionMotor.set(value);
  }

  /**Sets the belts to the Scoring mode*/
  public void SetScoringState() {}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
