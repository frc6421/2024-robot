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
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransitionArmSubsystem extends SubsystemBase{
  
  public static class TransitionArmConstants {
    public static final int ARMMOTORRIGHT_CAN_ID = 22;
    public static final int ARMMOTORLEFT_CAN_ID = 23;

    public static final double ARMMOTORRIGHT_CLIMB_KP = 0.064;
    public static final double ARMMOTORLEFT_CLIMB_KP = 0.064;

    public static final double ARMMOTORRIGHT_KP = 0.016;
    public static final double ARMMOTORRIGHT_KI = 0.0;
    public static final double ARMMOTORRIGHT_KD = 0.0;
    public static final double ARMMOTORRIGHT_KS = 0.342;
    public static final double ARMMOTORRIGHT_KG = (0.3759 - ARMMOTORRIGHT_KS); // voltage

    public static final double ARMMOTORLEFT_KP = ARMMOTORRIGHT_KP;
    public static final double ARMMOTORLEFT_KI = ARMMOTORRIGHT_KI;
    public static final double ARMMOTORLEFT_KD = ARMMOTORRIGHT_KD; 
    public static final double ARMMOTORLEFT_KS = ARMMOTORRIGHT_KS;
    public static final double ARMMOTORLEFT_KG = ARMMOTORRIGHT_KG;

    public static final int ARM_STATOR_CURRENT_LIMIT = 100;

    public static final float ARM_FORWARD_SOFT_LIMIT = 116;
    public static final float ARM_REVERSE_SOFT_LIMIT = -7;
    public static final double ARM_GEAR_RATIO = 3 * 3 * 5 * (48.0/17.0);

    public static final double ARM_GEAR_RATIO_CONVERSION = 360.0 / TransitionArmConstants.ARM_GEAR_RATIO; // degrees

    public static final double ARM_EXTENDED_CLIMB = 95;

    public static final double ARM_AMP_POSITION = 87.5;
  }

  // fields
  private final CANSparkFlex armMotorRight;
  private final CANSparkFlex armMotorLeft;

  private final SparkPIDController armRightPIDController;
  private final SparkPIDController armLeftPIDController;

  private final RelativeEncoder armRightEncoder;
  private final RelativeEncoder armLeftEncoder;

  /** Creates a new transitionArm. */
  public TransitionArmSubsystem() {

      // CAN IDs
      armMotorRight = new CANSparkFlex(TransitionArmConstants.ARMMOTORRIGHT_CAN_ID, MotorType.kBrushless);
      armMotorLeft = new CANSparkFlex(TransitionArmConstants.ARMMOTORLEFT_CAN_ID, MotorType.kBrushless);
      
      // Factory defaults
      armMotorRight.restoreFactoryDefaults();
      armMotorLeft.restoreFactoryDefaults();

      // Encoders
      armRightEncoder = armMotorRight.getEncoder();
      armLeftEncoder = armMotorLeft.getEncoder();

      armRightEncoder.setPositionConversionFactor(TransitionArmConstants.ARM_GEAR_RATIO_CONVERSION);
      armLeftEncoder.setPositionConversionFactor(TransitionArmConstants.ARM_GEAR_RATIO_CONVERSION);

      // PID controller
      armRightPIDController = armMotorRight.getPIDController();
      armLeftPIDController = armMotorLeft.getPIDController();

      // Set right PID values
      armRightPIDController.setP(TransitionArmConstants.ARMMOTORRIGHT_KP, 0);
      armRightPIDController.setI(TransitionArmConstants.ARMMOTORRIGHT_KI, 0);
      armRightPIDController.setD(TransitionArmConstants.ARMMOTORRIGHT_KD, 0);

      // Set left PID values
      armLeftPIDController.setP(TransitionArmConstants.ARMMOTORLEFT_KP, 0);
      armLeftPIDController.setI(TransitionArmConstants.ARMMOTORLEFT_KP, 0);
      armLeftPIDController.setD(TransitionArmConstants.ARMMOTORLEFT_KP, 0);

      // Set slot 1 right PID values
      armRightPIDController.setP(TransitionArmConstants.ARMMOTORRIGHT_CLIMB_KP, 1);
      armRightPIDController.setI(TransitionArmConstants.ARMMOTORRIGHT_KI, 1);
      armRightPIDController.setD(TransitionArmConstants.ARMMOTORRIGHT_KD, 1);

      // Set slot 1 left PID values
      armLeftPIDController.setP(TransitionArmConstants.ARMMOTORLEFT_CLIMB_KP, 1);
      armLeftPIDController.setI(TransitionArmConstants.ARMMOTORLEFT_KP, 1);
      armLeftPIDController.setD(TransitionArmConstants.ARMMOTORLEFT_KP, 1);

      // Inversions
      armMotorRight.setInverted(false);
      armMotorLeft.setInverted(true);

      // Idle Modes
      armMotorRight.setIdleMode(IdleMode.kBrake);
      armMotorLeft.setIdleMode(IdleMode.kBrake);

      // Current Limits
      armMotorRight.setSmartCurrentLimit(TransitionArmConstants.ARM_STATOR_CURRENT_LIMIT);
      armMotorLeft.setSmartCurrentLimit(TransitionArmConstants.ARM_STATOR_CURRENT_LIMIT);

      // Soft Limits
      armMotorRight.setSoftLimit(SoftLimitDirection.kForward, TransitionArmConstants.ARM_FORWARD_SOFT_LIMIT);
      armMotorRight.setSoftLimit(SoftLimitDirection.kReverse, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT);

      armMotorLeft.setSoftLimit(SoftLimitDirection.kForward, TransitionArmConstants.ARM_FORWARD_SOFT_LIMIT);
      armMotorLeft.setSoftLimit(SoftLimitDirection.kReverse, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT);

      armMotorLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
      armMotorLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);

      armMotorRight.enableSoftLimit(SoftLimitDirection.kForward, true);
      armMotorRight.enableSoftLimit(SoftLimitDirection.kReverse, true);

      // Follower
      armMotorLeft.follow(armMotorRight, true);

      armRightEncoder.setPosition(TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT);
      armLeftEncoder.setPosition(TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT);
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
    armRightPIDController.setReference(position, ControlType.kPosition, 0, (TransitionArmConstants.ARMMOTORRIGHT_KS + TransitionArmConstants.ARMMOTORRIGHT_KG) * Math.cos(getArmMotorPositionDeg()), ArbFFUnits.kVoltage);
    armLeftPIDController.setReference(position, ControlType.kPosition, 0, (TransitionArmConstants.ARMMOTORRIGHT_KS + TransitionArmConstants.ARMMOTORRIGHT_KG) * Math.cos(getArmMotorPositionDeg()), ArbFFUnits.kVoltage);
  }

  /**
   * Sets the arm motors to the position inputed
   * @param position the position to set the motors to
   * @param slot the PID slot to use
   */
  public void setArmMotorPosition(double position, int slot)
  {
    armRightPIDController.setReference(position, ControlType.kPosition, slot, (TransitionArmConstants.ARMMOTORRIGHT_KS + TransitionArmConstants.ARMMOTORRIGHT_KG) * Math.cos(getArmMotorPositionDeg()), ArbFFUnits.kVoltage);
    armLeftPIDController.setReference(position, ControlType.kPosition, slot, (TransitionArmConstants.ARMMOTORRIGHT_KS + TransitionArmConstants.ARMMOTORRIGHT_KG) * Math.cos(getArmMotorPositionDeg()), ArbFFUnits.kVoltage);
  }

  /**
   * Takes the average position of the arm encoders 
   * @return the position in degrees
   */
  public double getArmMotorPositionDeg()
  {
    return (armRightEncoder.getPosition() + armLeftEncoder.getPosition()) / 2;
  }

  public double getEncoderLeftPosition()
  {
    return armLeftEncoder.getPosition();
  }

  public double getEncoderRightPosition()
  {
    return armRightEncoder.getPosition();
  }
}
