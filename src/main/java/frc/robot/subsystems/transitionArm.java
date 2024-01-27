// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

    public static final float ARM_FORAWRD_SOFT_LIMIT = 0; // TODO needs to be determined
    public static final float ARM_REVERSE_SOFT_LIMIT = 0; // TODO needs to be determined

    public static final double ARM_GEAR_RATIO = 1; // TODO needs to be determined
  }

  // fields
  private final CANSparkFlex armMotorRight;
  private final CANSparkFlex armMotorLeft;

  private final SparkPIDController armRightPIDController;
  private final SparkPIDController armLeftPIDController;

  private final RelativeEncoder armRightEncoder;
  private final RelativeEncoder armLeftEncoder;

  /** Creates a new transitionArm. */
  public TransitionArm() {

      // CAN IDs
      armMotorRight = new CANSparkFlex(TransitionArmConstants.ARMMOTORRIGHT_CAN_ID, MotorType.kBrushless);
      armMotorLeft = new CANSparkFlex(TransitionArmConstants.ARMMOTORLEFT_CAN_ID, MotorType.kBrushless);

      // Encoders
      armRightEncoder = armMotorRight.getEncoder();
      armLeftEncoder = armMotorLeft.getEncoder();

      // PID controller
      armRightPIDController = armMotorRight.getPIDController();
      armLeftPIDController = armMotorLeft.getPIDController();

      // Set right PID values
      armRightPIDController.setP(TransitionArmConstants.ARMMOTORRIGHT_KP);
      armRightPIDController.setI(TransitionArmConstants.ARMMOTORRIGHT_KI);
      armRightPIDController.setD(TransitionArmConstants.ARMMOTORRIGHT_KD);

      // Set left PID values
      armLeftPIDController.setP(TransitionArmConstants.ARMMOTORLEFT_KP);
      armLeftPIDController.setI(TransitionArmConstants.ARMMOTORLEFT_KP);
      armLeftPIDController.setD(TransitionArmConstants.ARMMOTORLEFT_KP);

      // Factory defaults
      armMotorRight.restoreFactoryDefaults();
      armMotorLeft.restoreFactoryDefaults();

      // Inversions
      armMotorRight.setInverted(false);
      armMotorLeft.setInverted(false);

      // Idle Modes
      armMotorRight.setIdleMode(IdleMode.kCoast);
      armMotorLeft.setIdleMode(IdleMode.kCoast);

      // Current Limits
      armMotorRight.setSmartCurrentLimit(TransitionArmConstants.ARM_STATOR_CURRENT_LIMIT);
      armMotorLeft.setSmartCurrentLimit(TransitionArmConstants.ARM_STATOR_CURRENT_LIMIT);

      // Soft Limits
      armMotorRight.setSoftLimit(SoftLimitDirection.kForward, TransitionArmConstants.ARM_FORAWRD_SOFT_LIMIT);
      armMotorRight.setSoftLimit(SoftLimitDirection.kReverse, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT);

      armMotorLeft.setSoftLimit(SoftLimitDirection.kForward, TransitionArmConstants.ARM_FORAWRD_SOFT_LIMIT);
      armMotorLeft.setSoftLimit(SoftLimitDirection.kReverse, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT);

      // Follower
      armMotorLeft.follow(armMotorRight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the arm motors to the position inputed
   * @param position the position to set the motors to
   */
  public void setArmMotorPosition(double position)
  {
    armRightPIDController.setReference(position, ControlType.kPosition);
    armLeftPIDController.setReference(position, ControlType.kPosition);
  }

  //TODO determine if an error is needed in case of motor failure 

  /**
   * Returns the average position of the arm in rotations
   * @return the average position in rotations
   */
  public double getArmMotorPosition()
  {
    return (armRightEncoder.getPosition() + armLeftEncoder.getPosition()) / 2;
  }

  /**
   * Returns the averageposition of the arm in degrees
   * @return the average position in degrees
   */
  public double getArmMotorPositionDeg()
  {
    return ((armRightEncoder.getPosition() + armLeftEncoder.getPosition()) / 2) * 360;
  }

  // TODO Sendable
  // Send: position, P value (set)
}
