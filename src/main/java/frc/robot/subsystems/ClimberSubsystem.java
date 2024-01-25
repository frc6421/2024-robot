// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberSubsystem extends SubsystemBase {
  private static boolean stateFlipper = false;
  public static class ClimberConstants {
    public static final String RIO_NAME = "rio";

    // Can ID
    private static final int LEFT_CLIMBER_CAN_ID = 40;
    private static final int RIGHT_CLIMBER_CAN_ID = 41;

    // Velocity control
    public static final double CLIMBER_KS = 0.0;
    public static final double CLIMBER_KV = 0.0;
    public static final double CLIMBER_KP = 0.0;
    public static final double CLIMBER_KI = 0.0;
    public static final double CLIMBER_KD = 0.0;

  }
  //Create new
  private TalonFX leftClimberMotor;
  private TalonFX rightClimberMotor;
  private TalonFXConfiguration climberMotorConfig;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // Make new TalonFX Motors
    leftClimberMotor = new TalonFX(ClimberConstants.LEFT_CLIMBER_CAN_ID);
    rightClimberMotor = new TalonFX(ClimberConstants.RIGHT_CLIMBER_CAN_ID);
    climberMotorConfig = new TalonFXConfiguration();

    //Restore Factory defaults
    leftClimberMotor.getConfigurator().apply(climberMotorConfig);
    rightClimberMotor.getConfigurator().apply(climberMotorConfig);

    //Set motors to brake mode
    climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    climberMotorConfig.Slot0.kS = ClimberConstants.CLIMBER_KS;
    climberMotorConfig.Slot0.kV = ClimberConstants.CLIMBER_KV;
    climberMotorConfig.Slot0.kP = ClimberConstants.CLIMBER_KP;
    climberMotorConfig.Slot0.kI = ClimberConstants.CLIMBER_KI;
    climberMotorConfig.Slot0.kD = ClimberConstants.CLIMBER_KD;
  }
  /** Lowers the climber arms to a set position
   * 
   * @param value Used to set the position of the motors
   */
  public void SetClimberArm(double value) {
    if (stateFlipper) {
      leftClimberMotor.set(value);
      rightClimberMotor.set(value);
    }
    else {
      leftClimberMotor.set(-value);
      rightClimberMotor.set(-value);
    }

  }

  /** A method to make a special button input by flipping a bool 
   * @return stateFlipper
  */
  public static void buttonChange()
  {
    if(stateFlipper)
    {
      stateFlipper = false;
    }
    else
    {
      stateFlipper = true;
    }
  }
  @Override
  public void periodic() {

  }
}
