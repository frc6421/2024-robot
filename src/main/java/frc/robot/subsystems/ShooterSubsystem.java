// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkFlex;


public class ShooterSubsystem extends SubsystemBase {
    public static class ShooterConstants{
      public static final int TOP_SHOOTER_CAN_ID = 30;
      public static final int BOTTOM_SHOOTER_CAN_ID = 31;
      public static final int CURRENT_LIMIT = 80;

      public static final double SHOOTER_P = 0;
      public static final double SHOOTER_I = 0;
      public static final double SHOOTER_D = 0;
    }
    //Creating the objects for the motors and their encoders, respectivly
    private CANSparkFlex topShooterMotor;
    private CANSparkFlex bottomShooterMotor;

    private SparkPIDController topShooterPID;
    private SparkPIDController bottomShooterPID;

    private static RelativeEncoder topEncoder;
    private static RelativeEncoder bottomEncoder;

    private double positionMinOutput;
    private double positionMaxOutput;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    //Setting their CAN ID and the type
    topShooterMotor = new CANSparkFlex(ShooterConstants.TOP_SHOOTER_CAN_ID, MotorType.kBrushless);
    bottomShooterMotor = new CANSparkFlex(ShooterConstants.BOTTOM_SHOOTER_CAN_ID, MotorType.kBrushless);

    //Reseting them to defaults
    topShooterMotor.restoreFactoryDefaults();
    bottomShooterMotor.restoreFactoryDefaults();

    topShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    bottomShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);

    topShooterMotor.setInverted(false);
    topShooterMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);

    //Setting the bottom motor to follow
    bottomShooterMotor.follow(topShooterMotor, true);

    //Setting the encoders
    topEncoder = topShooterMotor.getEncoder();
    bottomEncoder = bottomShooterMotor.getEncoder();

    // PID Stuff... fun... \\
    positionMinOutput = -1;
    positionMaxOutput =  1;

    topShooterPID = topShooterMotor.getPIDController();
    bottomShooterPID = bottomShooterMotor.getPIDController();
    
    topShooterPID.setFeedbackDevice(topEncoder);
    bottomShooterPID.setFeedbackDevice(bottomEncoder);

    topShooterPID.setP(ShooterConstants.SHOOTER_P, 0);
    topShooterPID.setI(ShooterConstants.SHOOTER_I, 0);
    topShooterPID.setD(ShooterConstants.SHOOTER_D, 0);
    bottomShooterPID.setP(ShooterConstants.SHOOTER_P, 0);
    bottomShooterPID.setI(ShooterConstants.SHOOTER_I, 0);
    bottomShooterPID.setD(ShooterConstants.SHOOTER_D, 0);
        
    topShooterPID.setOutputRange(positionMinOutput, positionMaxOutput, 0);
    bottomShooterPID.setOutputRange(positionMinOutput, positionMaxOutput, 0);
    // And the end of the PID Stuff. yay. \\
  }
  /**
   * Runs the motors at a desired velocity
   * @param velocity the speed of which to run the motors
   */
  public void setMotorVelocity(double velocity){
    topShooterPID.setReference(velocity, CANSparkFlex.ControlType.kVelocity, 0, velocity, SparkPIDController.ArbFFUnits.kVoltage);
    bottomShooterPID.setReference(velocity, CANSparkFlex.ControlType.kVelocity, 0, velocity, SparkPIDController.ArbFFUnits.kVoltage);
  }

  /**
   * Runs the motors at a desired velocity
   * @param velocity the speed of which to run the motors
   */
  public void setTuningMotorVelocity(double velocity, double feedForward){
    topShooterPID.setReference(velocity, CANSparkFlex.ControlType.kVelocity, 0, feedForward, SparkPIDController.ArbFFUnits.kVoltage);
    bottomShooterPID.setReference(velocity, CANSparkFlex.ControlType.kVelocity, 0, feedForward, SparkPIDController.ArbFFUnits.kVoltage);
  }

  /**
   * Gets the current RPM of the top motor
   * @return rpm
   */
  public double getTopMotorVelocity(){
    return topEncoder.getVelocity();
  }

  /**
   * Gets the current RPM of the bottom motor
   * @return rpm
   */
  public double getBottomMotorVelocity(){
    return bottomEncoder.getVelocity();
  }

  /**
   * Changes the P value of the PID control loop of each motor
   * @param P the new P 
   */
  public void setP(double P){
    topShooterPID.setP(P,0);
    bottomShooterPID.setP(P,0);
  }

  /**
   * Set both motor voltages.
   * @param voltage voltages of the motor.
   */
  public void setTopVoltage(double voltage) {
    topShooterMotor.setVoltage(voltage);
  }

    /**
   * Set both motor voltages.
   * @param voltage voltages of the motor.
   */
  public void setBottomVoltage(double voltage) {
    bottomShooterMotor.setVoltage(voltage);
  }

  public double getTopMotorVoltage() {
    return topEncoder.getVelocity();
  }

  public double getBottomMotorVoltage() {
    return bottomEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
