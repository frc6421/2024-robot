// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class AngledShooterSubsystem extends SubsystemBase {
  public static class AngleConstants{
    public static final int ANGLE_CAN_ID = 32;
    public static final int CURRENT_LIMIT = 60;

    public static final double ANGLE_P = 0;
    public static final double ANGLE_I = 0;
    public static final double ANGLE_D = 0;

    public static final float MAXIMIMUM_SOFT_LIMIT = 0;
    public static final float MINNIMUM_SOFT_LIMIT = 0;
  }
  //Creating the object for the motor and encoder
  private CANSparkMax angleMotor;

  private SparkPIDController angleMotorPID;

  private static RelativeEncoder angleEncoder;

  private double positionMinOutput;
  private double positionMaxOutput;

  /** Creates a new AngledShooterSubsystem. */
  public AngledShooterSubsystem() {
    //Setting their CAN ID and the type
    angleMotor = new CANSparkMax(AngleConstants.ANGLE_CAN_ID, MotorType.kBrushless);

    //Reseting them to defaults
    angleMotor.restoreFactoryDefaults();

    angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    angleMotor.setInverted(false);
    angleMotor.setSmartCurrentLimit(AngleConstants.CURRENT_LIMIT);

    //Setting the encoders
    angleEncoder = angleMotor.getEncoder();

    // PID Stuff... fun... \\
    positionMinOutput = -1;
    positionMaxOutput =  1;

    angleMotorPID = angleMotor.getPIDController();
    
    angleMotorPID.setFeedbackDevice(angleEncoder);

    angleMotorPID.setP(AngleConstants.ANGLE_P, 0);
    angleMotorPID.setI(AngleConstants.ANGLE_I, 0);
    angleMotorPID.setD(AngleConstants.ANGLE_D, 0);

    angleMotor.setSoftLimit(SoftLimitDirection.kForward, AngleConstants.MAXIMIMUM_SOFT_LIMIT);
    angleMotor.setSoftLimit(SoftLimitDirection.kReverse, AngleConstants.MINNIMUM_SOFT_LIMIT);
    
    angleMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    angleMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
    angleMotorPID.setOutputRange(positionMinOutput, positionMaxOutput, 0);

    //Creating the Shuffleboard tab for testing
    Shuffleboard.getTab("Angle Motor Subsystem").add(this);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
