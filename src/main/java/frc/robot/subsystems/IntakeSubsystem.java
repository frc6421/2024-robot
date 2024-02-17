// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  public static class IntakeConstants {
    public static final int INTAKE_MOTOR_CAN_ID = 20;

    public static final int INTAKE_MOTOR_GEAR_RATIO = 1;

    //Current Limit
    public static final int INTAKE_STATOR_CURRENT_LIMIT = 80;

    public static final double INTAKE_IN_SPEED = 0.85 * 12;

    //TODO verify on actual robot
    public static final double INTAKE_OUT_SPEED = -0.85 * 12;
  }

  private CANSparkMax intakeMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

    //Factory defaults
    intakeMotor.restoreFactoryDefaults();

    // Set Current Limits
    intakeMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT);
    
    intakeMotor.setInverted(true);
  }

  /** Sets the intakeMotor output
   * @param value output to apply
   */
  public void setIntakeSpeed(double value) {
    intakeMotor.set(value);
  }

  public void setIntakeVoltage(double voltage)
  {
    intakeMotor.setVoltage(voltage);
  }

  /**
   * Stops the intakeMotor
   */
  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  //TODO Sendable
}
