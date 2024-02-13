// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
public class ClimberSubsystem extends SubsystemBase {

  public static class ClimberConstants {

    // Can ID
    private static final int LEFT_CLIMBER_CAN_ID = 40;
    private static final int RIGHT_CLIMBER_CAN_ID = 41;

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
    public static final float CLIMBER_FORWARD_SOFT_LIMIT_ROTATIONS = 100;

    // Current Limits
    //TODO Confirm Limits
    public static final int CLIMBER_STATOR_CURRENT_LIMIT = 50;

    // Command States
    public static enum climberState {
      EXTENDED,
      LOW
    }
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
    leftClimberMotor = new CANSparkFlex(ClimberConstants.LEFT_CLIMBER_CAN_ID, null);
    rightClimberMotor = new CANSparkFlex(ClimberConstants.RIGHT_CLIMBER_CAN_ID, null);

    // Factory default
    leftClimberMotor.restoreFactoryDefaults();
    leftClimberMotor.restoreFactoryDefaults();

    // PID Controller
    rightClimberPIDController = leftClimberMotor.getPIDController();
    leftClimberPIDController = rightClimberMotor.getPIDController();
    
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
    leftClimberMotor.setInverted(false);
    leftClimberMotor.setSmartCurrentLimit(ClimberConstants.CLIMBER_STATOR_CURRENT_LIMIT);
    rightClimberMotor.setInverted(false);
    rightClimberMotor.setSmartCurrentLimit(ClimberConstants.CLIMBER_STATOR_CURRENT_LIMIT);

    // Follower
    rightClimberMotor.follow(leftClimberMotor, false);

    // Apply PID
    leftClimberPIDController.setP(ClimberConstants.CLIMBER_KP, 0);
    leftClimberPIDController.setI(ClimberConstants.CLIMBER_KI, 0);
    leftClimberPIDController.setD(ClimberConstants.CLIMBER_KD, 0);
    leftClimberPIDController.setSmartMotionMaxVelocity(ClimberConstants.CLIMBER_KV, 0);

    rightClimberPIDController.setP(ClimberConstants.CLIMBER_KP, 0);
    rightClimberPIDController.setI(ClimberConstants.CLIMBER_KI, 0);
    rightClimberPIDController.setD(ClimberConstants.CLIMBER_KD, 0);
    rightClimberPIDController.setSmartMotionMaxVelocity(ClimberConstants.CLIMBER_KV, 0);

    // Apply gear ratio
    leftClimberEncoder.setPositionConversionFactor(360 / ClimberConstants.CLIMBER_GEAR_RATIO);
    rightClimberEncoder.setPositionConversionFactor(360 / ClimberConstants.CLIMBER_GEAR_RATIO);

    // Current limits
    leftClimberMotor.setSmartCurrentLimit(ClimberConstants.CLIMBER_STATOR_CURRENT_LIMIT);
    rightClimberMotor.setSmartCurrentLimit(ClimberConstants.CLIMBER_STATOR_CURRENT_LIMIT);
  }

  /** Sets the climber arms to a set position
   * 
   * @param position Used to set the position of the motors
   */
  public void setClimberMotorPosition(double position) {
    if (isClimberExtended) {
      leftClimberPIDController.setReference(0, CANSparkFlex.ControlType.kPosition);
      isClimberExtended = false;
    }
    else {
      leftClimberPIDController.setReference(position, CANSparkFlex.ControlType.kPosition);
      isClimberExtended = true;
    }
  }

  /** Returns a value in rotations of the current motor
   * @return  position as a double (the average position of both motors)
   */
  public double getClimberMotorPosition() {
    return (leftClimberEncoder.getPosition() + rightClimberEncoder.getPosition()) / 2;
  }


  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("SwerveModule");

    builder.addDoubleProperty("Climber P Value", () -> leftClimberPIDController.getP(), null);
    builder.addDoubleProperty("Climber I Value", () -> leftClimberPIDController.getI(), null);
    builder.addDoubleProperty("Climber D Value", () -> leftClimberPIDController.getD(), null);
    builder.addDoubleProperty("Motor Output", () -> leftClimberMotor.get(), null);
  }
}
