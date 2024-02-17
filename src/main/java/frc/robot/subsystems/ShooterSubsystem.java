// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class ShooterSubsystem extends SubsystemBase {
    public static class ShooterConstants{
      public static final int TOP_SHOOTER_CAN_ID = 30;
      public static final int BOTTOM_SHOOTER_CAN_ID = 31;
      public static final int CURRENT_LIMIT = 80;
      //TODO: Verify that this is the maximum speed the motors can achieve. 
      public static final int TOP_MAXIMUM_SPEED_IN_RPM = 6000;
      public static final int BOTTOM_MAXIMUM_SPEED_IN_RPM = 6000;

      public static final double TOP_KS = 0.34;
      public static final double TOP_KV = 0.130;
      public static final double TOP_KP = 0;
      public static final double TOP_KI = 0;
      public static final double TOP_KD = 0;

      public static final double BOTTOM_KS = 0.34;
      public static final double BOTTOM_KV = 0.130;
      public static final double BOTTOM_KP = 0;
      public static final double BOTTOM_KI = 0;
      public static final double BOTTOM_KD = 0;
    }
    //Creating the objects for the motors and their encoders, respectivly
    private TalonFX topShooterMotor;
    private TalonFX bottomShooterMotor;

    public TalonFXConfiguration topShooterConfig;
    public TalonFXConfiguration bottomShooterConfig;

    private VelocityVoltage shooterMotorVelocity;
    

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    //Setting their CAN ID and the type
    topShooterMotor = new TalonFX(ShooterConstants.TOP_SHOOTER_CAN_ID);
    bottomShooterMotor = new TalonFX(ShooterConstants.BOTTOM_SHOOTER_CAN_ID);

    topShooterConfig = new TalonFXConfiguration();
    bottomShooterConfig = new TalonFXConfiguration();

    shooterMotorVelocity = new VelocityVoltage(0);

    topShooterMotor.getConfigurator().apply(topShooterConfig);
    bottomShooterMotor.getConfigurator().apply(bottomShooterConfig);

    topShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    bottomShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    //TODO: Verify Inverts
    topShooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    bottomShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


    topShooterConfig.Slot0.kS = ShooterConstants.TOP_KS;
    topShooterConfig.Slot0.kV = ShooterConstants.TOP_KV;
    topShooterConfig.Slot0.kP = ShooterConstants.TOP_KP;
    topShooterConfig.Slot0.kI = ShooterConstants.TOP_KI;
    topShooterConfig.Slot0.kD = ShooterConstants.TOP_KD;

    bottomShooterConfig.Slot0.kS = ShooterConstants.BOTTOM_KS;
    bottomShooterConfig.Slot0.kV = ShooterConstants.BOTTOM_KV;
    bottomShooterConfig.Slot0.kP = ShooterConstants.BOTTOM_KP;
    bottomShooterConfig.Slot0.kI = ShooterConstants.BOTTOM_KI;
    bottomShooterConfig.Slot0.kD = ShooterConstants.BOTTOM_KD;

    topShooterConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    bottomShooterConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.CURRENT_LIMIT;

    topShooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    bottomShooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    topShooterMotor.getConfigurator().apply(topShooterConfig);
    bottomShooterMotor.getConfigurator().apply(bottomShooterConfig);
  }

  /**
   * Runs the motors at a desired velocity
   * @param velocity the speed of which to run the motors
   */
  public void setShooterMotorVelocity(double velocity){
    shooterMotorVelocity.withVelocity(velocity / 60);
    topShooterMotor.setControl(shooterMotorVelocity);
    bottomShooterMotor.setControl(shooterMotorVelocity);
  }

  
  public double getTopMotorVelocity(){
    return 60 * topShooterMotor.getVelocity().refresh().getValue();
  }

  public double getBottomMotorVelocity(){
    return 60 * bottomShooterMotor.getVelocity().refresh().getValue();
  }

  public void setTopConfig(double P){
    topShooterConfig.Slot0.kP = P;
    topShooterMotor.getConfigurator().apply(topShooterConfig);
  }

  public void setBottomConfig(double P){
    bottomShooterConfig.Slot0.kP = P;
    bottomShooterMotor.getConfigurator().apply(bottomShooterConfig);
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
    return topShooterMotor.getMotorVoltage().refresh().getValue();
  }

  public double getBottomMotorVoltage() {
    return bottomShooterMotor.getMotorVoltage().refresh().getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
