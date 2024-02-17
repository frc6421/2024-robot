// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;

public class ShooterAngleSubsystem extends SubsystemBase {
  public static class AngleConstants{
    public static final int ANGLE_CAN_ID = 32;
    public static final int CURRENT_LIMIT = 60;

    //TODO: Calibration for P value
    public static final double ANGLE_KP = 0.06;
    public static final double ANGLE_KI = 0;
    public static final double ANGLE_KD = 0;

    public static final double ANGLE_KS = 0.187;

    //TODO: Verify that the minimum is the extrusion bellow the shooter
    public static final float MAXIMIMUM_SOFT_LIMIT_DEGREES = 55;
    public static final float MINNIMUM_SOFT_LIMIT_DEGREES = -25;

    public static final int GEAR_RATIO = 180;
    public static final double DEGREES_PER_MOTOR_ROTATION = (360.0 / AngleConstants.GEAR_RATIO);

    public static enum angleState {
      MAX,  
      MID,
      MIN
    }
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

    angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    angleMotor.setInverted(true);
    angleMotor.setSmartCurrentLimit(AngleConstants.CURRENT_LIMIT);

    angleMotor.clearFaults();

    //Setting encoder with the conversion factor of the gearbox
    angleEncoder = angleMotor.getEncoder();
    angleEncoder.setPositionConversionFactor(AngleConstants.DEGREES_PER_MOTOR_ROTATION);

    angleEncoder.setPosition(AngleConstants.MINNIMUM_SOFT_LIMIT_DEGREES);

    // PID Stuff... fun... \\
    positionMinOutput = -1;
    positionMaxOutput =  1;

    angleMotorPID = angleMotor.getPIDController();
    
    angleMotorPID.setFeedbackDevice(angleEncoder);

    angleMotorPID.setP(AngleConstants.ANGLE_KP, 0);
    angleMotorPID.setI(AngleConstants.ANGLE_KI, 0);
    angleMotorPID.setD(AngleConstants.ANGLE_KD, 0);

    angleMotor.setSoftLimit(SoftLimitDirection.kForward, AngleConstants.MAXIMIMUM_SOFT_LIMIT_DEGREES);
    angleMotor.setSoftLimit(SoftLimitDirection.kReverse, AngleConstants.MINNIMUM_SOFT_LIMIT_DEGREES);
    
    angleMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    angleMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
    angleMotorPID.setOutputRange(positionMinOutput, positionMaxOutput, 0);

  }
  /**
   * Sets the output of the angle motor to go to a certain angle
   * @param angle The angle of which to set the motor to
   */
  public void setAngle(double angle){
    angleMotorPID.setReference(angle, CANSparkMax.ControlType.kPosition, 0, AngleConstants.ANGLE_KS, SparkPIDController.ArbFFUnits.kVoltage);
  }

  //TODO: Remove the following functions after testing!

  /**
   * Sets the new P value of the motor retrived from ShuffleBoard
   * @param P the new P value to set to
   */
  public void setP(double P){
    angleMotorPID.setP(P);
  }

  /**
   * Sets the new FF value of the motor retrived from ShuffleBoard
   * @param FF the new FF value to set to
   */
  private void setFF(double FF){
    angleMotorPID.setFF(FF);
  }
  
  /**
   * Gets the position of the encoder to compare to the actual value
   * @return Arm position, in degrees
   */
  public double getAngleEncoderPostition(){
    return angleEncoder.getPosition();
  }
  /**
   * Gets the velocity of the encoder to see if the motor is "jittering"
   * @return The Motor velocity, in RPM
   */
  private double getAngleEncoderVelocity(){
    return angleEncoder.getVelocity();
  }

  //Shuffleboard stuff. Don't ask.
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("AngleSubsystem");


    builder.addDoubleProperty("Set P", null, this::setP);
    builder.addDoubleProperty("Set FF", null, this::setFF);
    builder.addDoubleProperty("Set Angle", null, this::setAngle);

    builder.addDoubleProperty("Encoder Value", this::getAngleEncoderPostition, null);
    builder.addDoubleProperty("Encoder Velocity", this::getAngleEncoderVelocity, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorVoltage(double voltage) {
    angleMotor.setVoltage(voltage);
  }
}
