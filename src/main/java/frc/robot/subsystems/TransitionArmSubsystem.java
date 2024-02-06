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

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransitionArmSubsystem extends SubsystemBase implements Sendable {
  
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
    public static final double ARMMOTORRIGHT_KG = 0.3453;

    public static final double ARMMOTORLEFT_KP = 0.3; // TODO needs to be tuned
    public static final double ARMMOTORLEFT_KI = 0.0;
    public static final double ARMMOTORLEFT_KD = 0.0; 
    public static final double ARMMOTORLEFT_KG = 0.3453; // voltage

    // public static final double ARMMOTORLEFT_FF = 0; // TODO needs to be determined 
    // public static final double ARMMOTORRIGHT_FF = 0; // TODO needs to be determined 


    public static final int ARM_STATOR_CURRENT_LIMIT = 50;

    public static final float ARM_FORAWRD_SOFT_LIMIT = 100; // TODO needs to be determined
    public static final float ARM_REVERSE_SOFT_LIMIT = -7; // TODO needs to be determined

    public static final double ARM_GEAR_RATIO = 3 * 3 * 5 * (48.0/17.0); // TODO needs to be determined
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

      armRightEncoder.setPositionConversionFactor(360.0 / TransitionArmConstants.ARM_GEAR_RATIO);
      armLeftEncoder.setPositionConversionFactor(360.0 / TransitionArmConstants.ARM_GEAR_RATIO);

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
      armMotorRight.setSoftLimit(SoftLimitDirection.kForward, TransitionArmConstants.ARM_FORAWRD_SOFT_LIMIT);
      armMotorRight.setSoftLimit(SoftLimitDirection.kReverse, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT);

      armMotorLeft.setSoftLimit(SoftLimitDirection.kForward, TransitionArmConstants.ARM_FORAWRD_SOFT_LIMIT);
      armMotorLeft.setSoftLimit(SoftLimitDirection.kReverse, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT);

      armMotorLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
      armMotorLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);

      armMotorRight.enableSoftLimit(SoftLimitDirection.kForward, true);
      armMotorRight.enableSoftLimit(SoftLimitDirection.kReverse, true);

      // Follower
      armMotorLeft.follow(armMotorRight, true);

      armRightEncoder.setPosition(-7);
      armLeftEncoder.setPosition(-7);
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
   * Returns the averageposition of the arm in degrees
   * @return the average position in degrees
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

  public double getArmP()
  {
    return armRightPIDController.getP();
  }

  public void setArmP(double value)
  {
    armRightPIDController.setP(value);
    armLeftPIDController.setP(value);
  }

  public void setVoltage(double voltage)
  {
    armMotorLeft.setVoltage(voltage);
    armMotorRight.setVoltage(voltage);
  }
  // TODO Sendable
  // Send: position, P value (set)

  // @Override
  // public void initSendable(SendableBuilder builder) {
  //     super.initSendable(builder);
  // }
}
