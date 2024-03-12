// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

public class ClimberSubsystem extends SubsystemBase {

  public static class ClimberConstants {

    // Can ID
    private static final int LEFT_CLIMBER_CAN_ID = 41;
    private static final int RIGHT_CLIMBER_CAN_ID = 40;

    // Gear ratio 
    private static final double CLIMBER_GEAR_RATIO = 25;

    // PID
    public static final double CLIMBER_KS = -0.4;
    public static final double CLIMBER_KP = 0.02;
    public static final double CLIMBER_KI = 0.0;
    public static final double CLIMBER_KD = 0.0;
    public static final double CLIMBER_KG = -0.6;

    public static final double CLIMBER_CLIMB_IN_POS = 75;

    // Climber just under hooks: 2781
    // Climber new soft limit: 4396
    // Soft Limits
    public static final float CLIMBER_REVERSE_SOFT_LIMIT_ROTATIONS = 0; 
    public static final float CLIMBER_FORWARD_SOFT_LIMIT_ROTATIONS = 4396;

    // Current Limits
    public static final int CLIMBER_STATOR_CURRENT_LIMIT = 100;
  }

  //Create new Motors/Controllers
  private CANSparkFlex leftClimberMotor;
  private CANSparkFlex rightClimberMotor;

  private final SparkPIDController rightClimberPIDController;
  private final SparkPIDController leftClimberPIDController;

  private final RelativeEncoder leftClimberEncoder;
  private final RelativeEncoder rightClimberEncoder;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    // Make new NEO Motors
    leftClimberMotor = new CANSparkFlex(ClimberConstants.LEFT_CLIMBER_CAN_ID, MotorType.kBrushless);
    rightClimberMotor = new CANSparkFlex(ClimberConstants.RIGHT_CLIMBER_CAN_ID, MotorType.kBrushless);

    // Factory default
    leftClimberMotor.restoreFactoryDefaults();
    rightClimberMotor.restoreFactoryDefaults();

    // PID Controller
    rightClimberPIDController = rightClimberMotor.getPIDController();
    leftClimberPIDController = leftClimberMotor.getPIDController();
    
    // Set Up Encoders
    leftClimberEncoder = leftClimberMotor.getEncoder();
    rightClimberEncoder = rightClimberMotor.getEncoder();

    // Soft limits
    rightClimberMotor.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.CLIMBER_FORWARD_SOFT_LIMIT_ROTATIONS);
    rightClimberMotor.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.CLIMBER_REVERSE_SOFT_LIMIT_ROTATIONS);

    rightClimberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    rightClimberMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    leftClimberMotor.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.CLIMBER_FORWARD_SOFT_LIMIT_ROTATIONS);
    leftClimberMotor.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.CLIMBER_REVERSE_SOFT_LIMIT_ROTATIONS);

    leftClimberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    leftClimberMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    
    // Idle modes
    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);

    // Inversion(s)
    leftClimberMotor.setInverted(false);

    // Current Limits
    leftClimberMotor.setSmartCurrentLimit(ClimberConstants.CLIMBER_STATOR_CURRENT_LIMIT);
    rightClimberMotor.setSmartCurrentLimit(ClimberConstants.CLIMBER_STATOR_CURRENT_LIMIT);

    // Follower
    rightClimberMotor.follow(leftClimberMotor, true);

    // Apply PID
    leftClimberPIDController.setP(ClimberConstants.CLIMBER_KP, 0);
    leftClimberPIDController.setI(ClimberConstants.CLIMBER_KI, 0);
    leftClimberPIDController.setD(ClimberConstants.CLIMBER_KD, 0);

    rightClimberPIDController.setP(ClimberConstants.CLIMBER_KP, 0);
    rightClimberPIDController.setI(ClimberConstants.CLIMBER_KI, 0);
    rightClimberPIDController.setD(ClimberConstants.CLIMBER_KD, 0);

    // Apply gear ratio
    leftClimberEncoder.setPositionConversionFactor(ClimberConstants.CLIMBER_GEAR_RATIO);
    rightClimberEncoder.setPositionConversionFactor(ClimberConstants.CLIMBER_GEAR_RATIO);

    // Current limits
    leftClimberMotor.setSmartCurrentLimit(ClimberConstants.CLIMBER_STATOR_CURRENT_LIMIT);
    rightClimberMotor.setSmartCurrentLimit(ClimberConstants.CLIMBER_STATOR_CURRENT_LIMIT);

    // Zeros the motors
    leftClimberEncoder.setPosition(0);
    rightClimberEncoder.setPosition(0);


  }

  // /** 
  //  * Sets the climber arms to a set position
  //  * @param position Used to set the position of the motors
  //  */
  // public void setClimberMotorPositionNonGrav(double position) {
  //   leftClimberPIDController.setReference(position, CANSparkFlex.ControlType.kPosition, 0, ClimberConstants.CLIMBER_KS, ArbFFUnits.kVoltage);
  //   rightClimberPIDController.setReference(position, CANSparkFlex.ControlType.kPosition, 0, ClimberConstants.CLIMBER_KS, ArbFFUnits.kVoltage);
  // }

  /** 
   * Sets the climber arms to a set position
   * @param position Used to set the position of the motors
   */
  public void setClimberMotorPosition(double position) {
    leftClimberPIDController.setReference(position, CANSparkFlex.ControlType.kPosition, 0, (ClimberConstants.CLIMBER_KS + ClimberConstants.CLIMBER_KG) * Math.cos(getClimberLeftMotorPosition() * 360), ArbFFUnits.kVoltage);
    rightClimberPIDController.setReference(position, CANSparkFlex.ControlType.kPosition, 0, (ClimberConstants.CLIMBER_KS + ClimberConstants.CLIMBER_KG) * Math.cos(getClimberRightMotorPosition() * 360), ArbFFUnits.kVoltage);
  }

  public void setClimberMotorPosition(double leftPosition, double rightPosition) {
    leftClimberPIDController.setReference(leftPosition, CANSparkFlex.ControlType.kPosition, 0, (ClimberConstants.CLIMBER_KS + ClimberConstants.CLIMBER_KG) * Math.cos(getClimberLeftMotorPosition() * 360), ArbFFUnits.kVoltage);
    rightClimberPIDController.setReference(rightPosition, CANSparkFlex.ControlType.kPosition, 0, (ClimberConstants.CLIMBER_KS + ClimberConstants.CLIMBER_KG) * Math.cos(getClimberRightMotorPosition() * 360), ArbFFUnits.kVoltage);
  }

  public void setClimberMotorPrevPosition()
  {
    double leftPos = getClimberLeftMotorPosition();
    double rightPos = getClimberRightMotorPosition();

    setClimberMotorPosition(leftPos, rightPos);
  }

  public void stopClimbMotors()
  {
    leftClimberMotor.stopMotor();
    rightClimberMotor.stopMotor();
  }

  /** 
   * Returns a value in rotations of the climber motor(s)
   * @return  position as a double (the average position of both motors)
   */
  public double getClimberMotorPosition() {
    return (leftClimberEncoder.getPosition() + rightClimberEncoder.getPosition()) / 2;
  }

  /** 
   * Returns a value in rotations of the left motor
   * @return  position as a double
   */
  public double getClimberLeftMotorPosition() {
    return leftClimberEncoder.getPosition();
  }

  /** 
   * Returns a value in rotations of the left motor
   * @return  position as a double
   */
  public double getClimberRightMotorPosition() {
    return rightClimberEncoder.getPosition();
  }

  public void setP(double p) {
    leftClimberPIDController.setP(p, 0);
    rightClimberPIDController.setP(p, 0);
  }

  public void setVoltage(double voltage) {
    leftClimberMotor.setVoltage(voltage);
    rightClimberMotor.setVoltage(voltage);
  }
}