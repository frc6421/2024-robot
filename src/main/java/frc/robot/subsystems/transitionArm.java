// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class transitionArm extends SubsystemBase {
  
  public static class transitionArmConstants {
    
    public static enum armState {
      INTAKE,
      SCOREING
    }

    public static final int ARMMOTORRIGHT_CAN_ID = 22;
    public static final int ARMMOTORLEFT_CAN_ID = 23;

    public static final double ARMMOTORRIGHT_KP = 0.3; // needs to be tuned
    public static final double ARMMOTORRIGHT_KI = 0.0;
    public static final double ARMMOTORRIGHT_KD = 0.0;

    public static final double ARMMOTORLEFT_KP = 0.3; // needs to be tuned
    public static final double ARMMOTORLEFT_KI = 0.0;
    public static final double ARMMOTORLEFT_KD = 0.0; 
  }

  // feilds
  private final TalonFX armMotorRight;
  private final TalonFX armMotorLeft;

  private final TalonFXConfiguration armMotorRightConfig;
  private final TalonFXConfiguration armMotorLeftConfig;

  private final PositionVoltage armPosition;

  /** Creates a new transitionArm. */
  public transitionArm() {
    // CAN IDs
    armMotorRight = new TalonFX(transitionArmConstants.ARMMOTORRIGHT_CAN_ID);
    armMotorLeft = new TalonFX(transitionArmConstants.ARMMOTORLEFT_CAN_ID);

    // Creates new configurations for the motors
    armMotorRightConfig = new TalonFXConfiguration();
    armMotorLeftConfig = new TalonFXConfiguration();

    // Set to factory defults
    armMotorRight.getConfigurator().apply(new TalonFXConfiguration());
    
    // Neutral mode and Inversions
    armMotorRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armMotorRightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    // Sets PID values
    armMotorRightConfig.Slot0.kP = transitionArmConstants.ARMMOTORRIGHT_KP;
    armMotorRightConfig.Slot0.kI = transitionArmConstants.ARMMOTORRIGHT_KI;
    armMotorRightConfig.Slot0.kD = transitionArmConstants.ARMMOTORRIGHT_KD;

    // set the new configutarion to the motor
    armMotorRight.getConfigurator().apply(armMotorRightConfig);

    // Set to factory defults
    armMotorLeft.getConfigurator().apply(new TalonFXConfiguration());

    // Neutral mode and Inversions
    armMotorLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armMotorLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    // Sets PID values
    armMotorLeftConfig.Slot0.kP = transitionArmConstants.ARMMOTORLEFT_KP;
    armMotorLeftConfig.Slot0.kI = transitionArmConstants.ARMMOTORLEFT_KI;
    armMotorLeftConfig.Slot0.kD = transitionArmConstants.ARMMOTORLEFT_KD;

    // set the new configutarion to the motor
    armMotorLeft.getConfigurator().apply(armMotorLeftConfig);

    // Set the left motor to follow the right motor
    armMotorLeft.setControl(new Follower(armMotorRight.getDeviceID(), false));

    armPosition = new PositionVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Returns the position of the right arm motor. The left motor follows the right motor.
   * @return position of the arm motors
   */
  public double getArmMotorPosition() {
    return armMotorRight.getPosition().refresh().getValue();
  }

 /**
   * sets the position of the right arm motor. The left motor follows the right motor.
   * @param position the position in rotations to set the motor to.
   */
  public void setArmMotorPosition(double position) {
    armPosition.Position = position;
    armMotorRight.setControl(armPosition);
  }
}
