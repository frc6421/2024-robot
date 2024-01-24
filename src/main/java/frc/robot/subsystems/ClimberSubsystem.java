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
  public static class ClimberConstants {
    public static final String RIO_NAME = "rio";

    private static final int LEFT_CLIMBER_CAN_ID = 40;
    private static final int RIGHT_CLIMBER_CAN_ID = 41;
    // Velocity control ???
    // public static final double CLIMBER_KS = 0.0;
    // public static final double CLIMBER_KV = 0.0;
    // public static final double CLIMBER_KP = 0.0;
    // public static final double CLIMBER_KI = 0.0;
    // public static final double CLIMBER_KD = 0.0;
  }
  
  private TalonFX leftClimberMotor;
  private TalonFX rightClimberMotor;
  private TalonFXConfiguration climberMotorConfig;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    leftClimberMotor = new TalonFX(ClimberConstants.LEFT_CLIMBER_CAN_ID);
    rightClimberMotor = new TalonFX(ClimberConstants.RIGHT_CLIMBER_CAN_ID);
    climberMotorConfig = new TalonFXConfiguration();

    //Restore Factory defaults
    leftClimberMotor.getConfigurator().apply(climberMotorConfig);
    rightClimberMotor.getConfigurator().apply(climberMotorConfig);

    //Set motors to brake mode
    climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    //Velocity control ???
    // climberMotorConfig.Slot0.kS = ClimberConstants.DRIVE_KS;
    // climberMotorConfig.Slot0.kV = ClimberConstants.DRIVE_KV;
    // climberMotorConfig.Slot0.kP = ClimberConstants.DRIVE_KP;
    // climberMotorConfig.Slot0.kI = ClimberConstants.DRIVE_KI;
    // climberMotorConfig.Slot0.kD = ClimberConstants.DRIVE_KD;
  }

  public void RaiseClimberArm(double value) {
    climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leftClimberMotor.set(value);
    rightClimberMotor.set(value);
  }

  public void LowerClimberArm(double value) {
    leftClimberMotor.set(-value);
    rightClimberMotor.set(-value);
  }
  @Override
  public void periodic() {

  }
}
