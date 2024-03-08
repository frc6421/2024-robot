// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;

public class ShooterAngleSubsystem extends SubsystemBase {
  public static class AngleConstants{
    public static final int ANGLE_CAN_ID = 32;
    public static final int CURRENT_LIMIT = 60;

    //PID values
    public static final double ANGLE_KP = 0.16;
    public static final double ANGLE_KI = 0;
    public static final double ANGLE_KD = 0;

    //Limits
    public static final float MAXIMIMUM_SOFT_LIMIT_DEGREES = 49;
    public static final float MINIMUM_SOFT_LIMIT_DEGREES = -25;

    //Encoder conversions
    public static final int GEAR_RATIO = 180;
    public static final double DEGREES_PER_MOTOR_ROTATION = (360.0 / AngleConstants.GEAR_RATIO);

    public static final double[] PIVOT_ANGLE = {45, 41, 38, 35, 33, 29.5, 28, 26.5, 25.5, 24.5, 23, 46};

  }
  //Creating the object for the motor and encoder
  private CANSparkMax angleMotor;

  private SparkPIDController angleMotorPID;

  private static RelativeEncoder angleEncoder;

  private double positionMinOutput;
  private double positionMaxOutput;

  /** Creates a new AngledShooterSubsystem. */
  public ShooterAngleSubsystem() {
    //Setting CAN ID and the type
    angleMotor = new CANSparkMax(AngleConstants.ANGLE_CAN_ID, MotorType.kBrushless);

    //Reseting to defaults
    angleMotor.restoreFactoryDefaults();

    //Set the idle mode of the motor
    angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    //Inverts
    angleMotor.setInverted(true);

    //Current limit
    angleMotor.setSmartCurrentLimit(AngleConstants.CURRENT_LIMIT);

    //Clearing faults
    angleMotor.clearFaults();

    //Setting encoder with the conversion factor of the gearbox
    angleEncoder = angleMotor.getEncoder();
    angleEncoder.setPositionConversionFactor(AngleConstants.DEGREES_PER_MOTOR_ROTATION);

    //Reseting the encoder to 0
    angleEncoder.setPosition(AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES);

    //How much of the output range one wants to use
    positionMinOutput = -1;
    positionMaxOutput =  1;

    //Getting the motor for PID
    angleMotorPID = angleMotor.getPIDController();
    
    //Setting the feedback device of the motor to the built-in encoder
    angleMotorPID.setFeedbackDevice(angleEncoder);

    //Setting motor PID values
    angleMotorPID.setP(AngleConstants.ANGLE_KP, 0);
    angleMotorPID.setI(AngleConstants.ANGLE_KI, 0);
    angleMotorPID.setD(AngleConstants.ANGLE_KD, 0);

    //Setting and enabling the soft limits of the motors
    angleMotor.setSoftLimit(SoftLimitDirection.kForward, AngleConstants.MAXIMIMUM_SOFT_LIMIT_DEGREES);
    angleMotor.setSoftLimit(SoftLimitDirection.kReverse, AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES);
    angleMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    angleMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    angleMotorPID.setOutputRange(positionMinOutput, positionMaxOutput, 0);
  }


  /**
   * Sets the output of the angle motor to go to a certain angle
   * @param angle The angle of which to set the motor to
   */
  public void setAngle(double angle){
    angleMotorPID.setReference(angle, CANSparkMax.ControlType.kPosition, 0, 0, SparkPIDController.ArbFFUnits.kVoltage);
  }


  /**
   * Gets the position of the encoder to compare to the actual value
   * @return Arm position, in degrees
   */
  public double getAngleEncoderPosition(){
    return angleEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Current Angle", getAngleEncoderPosition());
  }

  // @Override
  // public void initSendable(SendableBuilder builder) {
  //     // TODO Auto-generated method stub
  //     super.initSendable(builder);

  //     builder.addDoubleProperty("Get Pivot Angle", this::getAngleEncoderPosition, null);
  //     builder.addDoubleProperty("Set Pivot Angle", null, this::setAngle);
  // }
}
