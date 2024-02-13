// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.util.sendable.SendableBuilder;
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

    // TODO Limit switch sensors??? (boolean)
    // TODO Value is TBD
    private static final double CLIMBER_GEAR_RATIO = 1;
    // Velocity control
    // TODO Put in PID values
    public static final double CLIMBER_KS = 0.0;
    public static final double CLIMBER_KV = 0.0;
    public static final double CLIMBER_KP = 0.0;
    public static final double CLIMBER_KI = 0.0;
    public static final double CLIMBER_KD = 0.0;
    public static final double CLIMBER_KG = 0.0;


    //TODO Confirm values
    public static final float CLIMBER_REVERSE_SOFT_LIMIT_ROTATIONS = 0;
    public static final float CLIMBER_FORWARD_SOFT_LIMIT_ROTATIONS = 5;

    // Current Limits
    //TODO Confirm Limits
    public static final int CLIMBER_STATOR_CURRENT_LIMIT = 50;
  }

  //Create new Motors/Controllers
  private CANSparkFlex leftClimberMotor;
  private CANSparkFlex rightClimberMotor;

  private final SparkPIDController rightClimberPIDController;
  private final SparkPIDController leftClimberPIDController;

  private final RelativeEncoder leftClimberEncoder;
  private final RelativeEncoder rightClimberEncoder;

  public boolean isClimberExtended;

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

    // Set bool to false
    isClimberExtended = false;

    // Soft limits
    rightClimberMotor.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.CLIMBER_FORWARD_SOFT_LIMIT_ROTATIONS);
    rightClimberMotor.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.CLIMBER_REVERSE_SOFT_LIMIT_ROTATIONS);

    leftClimberMotor.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.CLIMBER_FORWARD_SOFT_LIMIT_ROTATIONS);
    leftClimberMotor.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.CLIMBER_REVERSE_SOFT_LIMIT_ROTATIONS);
    
    // Set motors to brake mode and set direction
    // TODO Confirm which direction config should be
    leftClimberMotor.setIdleMode(IdleMode.kCoast);
    leftClimberMotor.setInverted(false);
    leftClimberMotor.setSmartCurrentLimit(ClimberConstants.CLIMBER_STATOR_CURRENT_LIMIT);
    rightClimberMotor.setIdleMode(IdleMode.kCoast);
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

    leftClimberEncoder.setPosition(0);
    rightClimberEncoder.setPosition(0);
  }

  /** Sets the climber arms to a set position
   * 
   * @param position Used to set the position of the motors
   */
  public void setClimberMotorPosition(double position) {
    leftClimberPIDController.setReference(position, CANSparkFlex.ControlType.kPosition);
    rightClimberPIDController.setReference(position, CANSparkFlex.ControlType.kPosition);
    // if (isClimberExtended) {
    //   leftClimberPIDController.setReference(0, CANSparkFlex.ControlType.kPosition, 0, ClimberConstants.CLIMBER_KV, ArbFFUnits.kVoltage);
    //   isClimberExtended = false;
    // }
    // else {
    //   leftClimberPIDController.setReference(position, CANSparkFlex.ControlType.kPosition, 0, ClimberConstants.CLIMBER_KV, ArbFFUnits.kVoltage);
    //   isClimberExtended = true;
    // }
  }

  /** Returns a value in rotations of the current motor
   * @return  position as a double (the average position of both motors)
   */
  public double getClimberMotorPosition() {
    return (leftClimberEncoder.getPosition() + rightClimberEncoder.getPosition()) / 2;
  }

  public double getClimberLeftMotorPosition() {
    return leftClimberEncoder.getPosition();
  }

  public double getClimberRightMotorPosition() {
    return rightClimberEncoder.getPosition();
  }

  public void setClimberP(double p)
  {
    leftClimberPIDController.setP(p);
    rightClimberPIDController.setP(p);
  }

  public void setClimberVoltage(double v)
  {
    leftClimberMotor.setVoltage(v);
    rightClimberMotor.setVoltage(v);
  }

  // public void initSendable(SendableBuilder builder){
  //   builder.setSmartDashboardType("Climber");

  //   builder.addDoubleProperty("Climber P Value", () -> leftClimberPIDController.getP(), null);
  //   builder.addDoubleProperty("Climber Motor Position", () -> getClimberMotorPosition(), null);
  // }
}