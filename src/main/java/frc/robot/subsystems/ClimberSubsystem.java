// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;

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
    public static final double CLIMBER_REVERSE_SOFT_LIMIT_ROTATIONS = 0;
    public static final double CLIMBER_FORWARD_SOFT_LIMIT_ROTATIONS = 100;

    // Current Limits
    //TODO Confirm Limits
    public static final double CLIMBER_STATOR_CURRENT_LIMIT = 50;

    // Command States
    public static enum climberState {
      EXTENDED,
      LOW
    }
  }

  // TODO Trapezoid Profile (Maybe in command)
  //Create new
  private PositionVoltage climberPosition;
  private CANSparkFlex leftClimberMotor;
  private CANSparkFlex rightClimberMotor;
  private TalonFXConfiguration climberMotorConfig;
  private final SparkPIDController armRightPIDController;
  private final SparkPIDController armLeftPIDController;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // Make new TalonFX Motors
    leftClimberMotor = new CANSparkFlex(ClimberConstants.LEFT_CLIMBER_CAN_ID, null);
    rightClimberMotor = new CANSparkFlex(ClimberConstants.RIGHT_CLIMBER_CAN_ID, null);

    //Restore Factory defaults
    

    //Set motors to brake mode and set direction
    // TODO Confirm which direction config should be
    climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // apply PID
    climberMotorConfig.Slot0.kS = ClimberConstants.CLIMBER_KS;
    climberMotorConfig.Slot0.kV = ClimberConstants.CLIMBER_KV;
    climberMotorConfig.Slot0.kP = ClimberConstants.CLIMBER_KP;
    climberMotorConfig.Slot0.kI = ClimberConstants.CLIMBER_KI;
    climberMotorConfig.Slot0.kD = ClimberConstants.CLIMBER_KD;
    climberMotorConfig.Slot0.kG = ClimberConstants.CLIMBER_KG;

    // Apply soft limits
    climberMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climberMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.CLIMBER_FORWARD_SOFT_LIMIT_ROTATIONS;

    climberMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    climberMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.CLIMBER_REVERSE_SOFT_LIMIT_ROTATIONS;

    // Gear Ratio 
    climberMotorConfig.Feedback.SensorToMechanismRatio = ClimberConstants.CLIMBER_GEAR_RATIO;

    //Current limits
    climberMotorConfig.CurrentLimits.StatorCurrentLimit = ClimberConstants.CLIMBER_STATOR_CURRENT_LIMIT;

    // Apply Configurator
    leftClimberMotor.getConfigurator().apply(climberMotorConfig);
    
    // Change invert for follow motor
    climberMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rightClimberMotor.getConfigurator().apply(climberMotorConfig);

    // Set Left as master
    rightClimberMotor.setControl(new Follower(leftClimberMotor.getDeviceID(), false));

    // Set position voltage
    climberPosition = new PositionVoltage(ClimberConstants.CLIMBER_REVERSE_SOFT_LIMIT_ROTATIONS);
  }

  /** Sets the climber arms to a set position
   * 
   * @param position Used to set the position of the motors
   */
  public void setClimberMotorPosition(double position) {
      climberPosition.Position = position;
      rightClimberMotor.setControl(climberPosition);
  }

  /** Returns a value in rotations of the current motor
   * @return
   */
  public double getClimberMotorPosition() {
    return rightClimberMotor.getPosition().refresh().getValue();
  }
}
