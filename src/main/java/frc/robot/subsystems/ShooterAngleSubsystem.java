// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ShooterAngleSubsystem extends SubsystemBase {
  public static class AngleConstants{
    public static final int ANGLE_CAN_ID = 32;
    public static final int CURRENT_LIMIT = 60;

    public static final double ANGLE_P = 0;
    public static final double ANGLE_I = 0;
    public static final double ANGLE_D = 0;

    public static final float MAXIMIMUM_SOFT_LIMIT_DEGREES = 55;
    public static final float MINNIMUM_SOFT_LIMIT_DEGREES = -25;

    public static final int GEAR_RATIO = 180;
    public static final double DEGREES_PER_MOTOR_ROTATION = (360.0 / AngleConstants.GEAR_RATIO);

    public static final double MAX_ANGLE_GRAVITY_FF = 0;
  }
  //Creating the object for the motor and encoder
  private CANSparkMax angleMotor;

  private SparkPIDController angleMotorPID;

  private static RelativeEncoder angleEncoder;

  private double positionMinOutput;
  private double positionMaxOutput;

  private double angleDynamicFF;
  private double tempGravityFF;
  private double gravityOffset;

  /** Creates a new AngledShooterSubsystem. */
  public ShooterAngleSubsystem() {
    //Setting CAN ID and the type
    angleMotor = new CANSparkMax(AngleConstants.ANGLE_CAN_ID, MotorType.kBrushless);

    //Reseting to defaults
    angleMotor.restoreFactoryDefaults();

    angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    angleMotor.setInverted(false);
    angleMotor.setSmartCurrentLimit(AngleConstants.CURRENT_LIMIT);

    //Setting encoder with the conversion factor of the gearbox
    angleEncoder = angleMotor.getEncoder();
    angleEncoder.setPositionConversionFactor(AngleConstants.DEGREES_PER_MOTOR_ROTATION);

    // PID Stuff... fun... \\
    positionMinOutput = -1;
    positionMaxOutput =  1;

    angleMotorPID = angleMotor.getPIDController();
    
    angleMotorPID.setFeedbackDevice(angleEncoder);

    angleMotorPID.setP(AngleConstants.ANGLE_P, 0);
    angleMotorPID.setI(AngleConstants.ANGLE_I, 0);
    angleMotorPID.setD(AngleConstants.ANGLE_D, 0);

    angleMotor.setSoftLimit(SoftLimitDirection.kForward, AngleConstants.MAXIMIMUM_SOFT_LIMIT_DEGREES);
    angleMotor.setSoftLimit(SoftLimitDirection.kReverse, AngleConstants.MINNIMUM_SOFT_LIMIT_DEGREES);
    
    angleMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    angleMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
    angleMotorPID.setOutputRange(positionMinOutput, positionMaxOutput, 0);

    //Creating the Shuffleboard tab for testing
    Shuffleboard.getTab("Angle Motor Subsystem").add(this);
  }

  public void setPosition(double position){
    angleMotorPID.setReference(position, CANSparkMax.ControlType.kPosition, 0, position, SparkPIDController.ArbFFUnits.kPercentOut);
  }

  public void setAngle(double angle){
    setGravityOffset();
    angleMotorPID.setReference(angle, CANSparkMax.ControlType.kPosition, 0, angleDynamicFF, SparkPIDController.ArbFFUnits.kPercentOut);
  }

  public void setGravityOffset(){
    angleDynamicFF = (tempGravityFF * Math.cos(Math.toRadians(angleEncoder.getPosition())));
  }

  public void changeGravityOffset(){
    tempGravityFF = gravityOffset;
  }

  private void setP(double P){
    angleMotorPID.setP(P);
  }

  private void setFF(double FF){
    angleMotorPID.setFF(FF);
  }
  
  private void setTempGravityOffset(double tempGravityOffset){
    gravityOffset = tempGravityOffset;
  }

  private double getEncoderPostition(){
    return angleEncoder.getPosition();
  }

  private double getEncoderVelocity(){
    return angleEncoder.getVelocity();
  }

  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("AngleSubsystem");


    builder.addDoubleProperty("Set P", null, this::setP);
    builder.addDoubleProperty("Set FF", null, this::setFF);
    builder.addDoubleProperty("Set Gravity Offset", null, this::setTempGravityOffset);
    builder.addDoubleProperty("Set Angle", null, this::setAngle);

    builder.addDoubleProperty("Encoder Value", this::getEncoderPostition, null);
    builder.addDoubleProperty("Encoder Velocity", this::getEncoderVelocity, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
