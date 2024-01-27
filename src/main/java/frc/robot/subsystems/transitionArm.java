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

public class TransitionArm extends SubsystemBase {
  
  public static class TransitionArmConstants {
    
    // used for testing
    public static enum armState {
      INTAKE,
      SCORING
    }

    public static final int ARMMOTORRIGHT_CAN_ID = 22;
    public static final int ARMMOTORLEFT_CAN_ID = 23;

    public static final double ARMMOTORRIGHT_KP = 0.3; // TODO needs to be tuned
    public static final double ARMMOTORRIGHT_KI = 0.0;
    public static final double ARMMOTORRIGHT_KD = 0.0;
    public static final double ARMMOTORRIGHT_KG = 0.0;

    public static final double ARMMOTORLEFT_KP = 0.3; // TODO needs to be tuned
    public static final double ARMMOTORLEFT_KI = 0.0;
    public static final double ARMMOTORLEFT_KD = 0.0; 
    public static final double ARMMOTORLEFT_KG = 0.0;

    public static final int ARM_STATOR_CURRENT_LIMIT = 50;

    public static final double ARM_FORAWRD_SOFT_LIMIT = 0; // TODO needs to be determined
    public static final double ARM_REVERSE_SOFT_LIMIT = 0; // TODO needs to be determined

    public static final double ARM_GEAR_RATIO = 1; // TODO needs to be determined
  }

  // fields
  private final TalonFX armMotorRight;
  private final TalonFX armMotorLeft;

  private final TalonFXConfiguration armMotorRightConfig;
  private final TalonFXConfiguration armMotorLeftConfig;

  private final PositionVoltage armPosition;

  /** Creates a new transitionArm. */
  public TransitionArm() {

    // CAN IDs
    armMotorRight = new TalonFX(TransitionArmConstants.ARMMOTORRIGHT_CAN_ID);
    armMotorLeft = new TalonFX(TransitionArmConstants.ARMMOTORLEFT_CAN_ID);

    // Creates new configurations for the motors
    armMotorRightConfig = new TalonFXConfiguration();
    armMotorLeftConfig = new TalonFXConfiguration();

    // Set to factory defults
    armMotorRight.getConfigurator().apply(new TalonFXConfiguration());
    
    // Neutral mode and Inversions
    armMotorRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armMotorRightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Soft limits
    armMotorRightConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    armMotorRightConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    armMotorRightConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TransitionArmConstants.ARM_FORAWRD_SOFT_LIMIT;
    armMotorRightConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT;
    
    // Sets PID values
    armMotorRightConfig.Slot0.kP = TransitionArmConstants.ARMMOTORRIGHT_KP;
    armMotorRightConfig.Slot0.kI = TransitionArmConstants.ARMMOTORRIGHT_KI;
    armMotorRightConfig.Slot0.kD = TransitionArmConstants.ARMMOTORRIGHT_KD;
    armMotorRightConfig.Slot0.kG = TransitionArmConstants.ARMMOTORRIGHT_KG;

    // Current limit
    armMotorRightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    armMotorRightConfig.CurrentLimits.StatorCurrentLimit = TransitionArmConstants.ARM_STATOR_CURRENT_LIMIT;

    // Gear ratio
    armMotorRightConfig.Feedback.SensorToMechanismRatio = TransitionArmConstants.ARM_GEAR_RATIO;

    // Set the new configutarion to the motor
    armMotorRight.getConfigurator().apply(armMotorRightConfig);

    // Set to factory defults
    armMotorLeft.getConfigurator().apply(new TalonFXConfiguration());

    // Neutral mode and Inversions
    armMotorLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armMotorLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Soft limits
    armMotorLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    armMotorLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    armMotorLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TransitionArmConstants.ARM_FORAWRD_SOFT_LIMIT;
    armMotorLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT;
    
    // Sets PID values
    armMotorLeftConfig.Slot0.kP = TransitionArmConstants.ARMMOTORLEFT_KP;
    armMotorLeftConfig.Slot0.kI = TransitionArmConstants.ARMMOTORLEFT_KI;
    armMotorLeftConfig.Slot0.kD = TransitionArmConstants.ARMMOTORLEFT_KD;
    armMotorLeftConfig.Slot0.kG = TransitionArmConstants.ARMMOTORLEFT_KG;

    // Current limit
    armMotorLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    armMotorLeftConfig.CurrentLimits.StatorCurrentLimit = TransitionArmConstants.ARM_STATOR_CURRENT_LIMIT;

    // Gear ratio
    armMotorRightConfig.Feedback.SensorToMechanismRatio = TransitionArmConstants.ARM_GEAR_RATIO;

    // Set the new configutarion to the motor
    armMotorLeft.getConfigurator().apply(armMotorLeftConfig);

    // Set the left motor to follow the right motor
    armMotorLeft.setControl(new Follower(armMotorRight.getDeviceID(), false));

    armPosition = new PositionVoltage(TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT);
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
   * Sets the position of the right arm motor. The left motor follows the right motor.
   * @param position the position in rotations to set the motor to.
   */
  public void setArmMotorPosition(double position) {
    armPosition.Position = position;
    armMotorRight.setControl(armPosition);
  }

  // TODO Sendable
  // Send: position, P value (set)
}
