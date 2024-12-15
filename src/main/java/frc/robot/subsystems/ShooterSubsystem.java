// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Cameras;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotStates;

import java.util.Optional;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterSubsystem extends SubsystemBase {
  public static class ShooterConstants {
    public static final int TOP_SHOOTER_CAN_ID = 30;
    public static final int BOTTOM_SHOOTER_CAN_ID = 31;
    public static final int CURRENT_LIMIT = 80;

    public static final double TOP_KS = 0.34;
    public static final double TOP_KV = 0.130;
    public static final double TOP_KP = 0.45;
    public static final double TOP_KI = 0;
    public static final double TOP_KD = 0;

    public static final double BOTTOM_KS = 0.4;
    public static final double BOTTOM_KV = 0.130;
    public static final double BOTTOM_KP = 0.45;
    public static final double BOTTOM_KI = 0;
    public static final double BOTTOM_KD = 0;

    public static final double SHOOTER_SHUTTLE_RPM = 4300;

  }

  // Creating the objects for the motors and their encoders, respectively
  public TalonFX topShooterMotor;
  private TalonFX bottomShooterMotor;

  private TalonFXConfiguration topShooterConfig;
  private TalonFXConfiguration bottomShooterConfig;

  private VelocityVoltage shooterMotorVelocity;

  public double targetShooterRPM = 0.0;

  private int targetTagID = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    // Setting their CAN ID and the type
    topShooterMotor = new TalonFX(ShooterConstants.TOP_SHOOTER_CAN_ID);
    bottomShooterMotor = new TalonFX(ShooterConstants.BOTTOM_SHOOTER_CAN_ID);

    // Creating the motor configs
    topShooterConfig = new TalonFXConfiguration();
    bottomShooterConfig = new TalonFXConfiguration();

    // For setting the velocity
    shooterMotorVelocity = new VelocityVoltage(0);

    // Resetting motors to factory defaults
    topShooterMotor.getConfigurator().apply(topShooterConfig);
    bottomShooterMotor.getConfigurator().apply(bottomShooterConfig);

    // Mode of the motor
    topShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    bottomShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Motor Inverts
    topShooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    bottomShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Setting the PID, kS, and kV of the top motor
    topShooterConfig.Slot0.kS = ShooterConstants.TOP_KS;
    topShooterConfig.Slot0.kV = ShooterConstants.TOP_KV;
    topShooterConfig.Slot0.kP = ShooterConstants.TOP_KP;
    topShooterConfig.Slot0.kI = ShooterConstants.TOP_KI;
    topShooterConfig.Slot0.kD = ShooterConstants.TOP_KD;

    // Setting the PID, kS, and kV of the bottom motor
    bottomShooterConfig.Slot0.kS = ShooterConstants.BOTTOM_KS;
    bottomShooterConfig.Slot0.kV = ShooterConstants.BOTTOM_KV;
    bottomShooterConfig.Slot0.kP = ShooterConstants.BOTTOM_KP;
    bottomShooterConfig.Slot0.kI = ShooterConstants.BOTTOM_KI;
    bottomShooterConfig.Slot0.kD = ShooterConstants.BOTTOM_KD;

    // Current limits
    topShooterConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    bottomShooterConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    topShooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    bottomShooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Applying the changes to the motors
    topShooterMotor.getConfigurator().apply(topShooterConfig);
    bottomShooterMotor.getConfigurator().apply(bottomShooterConfig);

    // Shuffleboard tab
    Shuffleboard.getTab("Shooter").add(this);

  }

  /**
   * Runs the motors at a desired velocity
   * 
   * @param velocity the speed of which to run the motors, in RPM
   */
  public void setShooterMotorVelocity(double velocity) {
    targetShooterRPM = velocity;
    setTopShooterMotorVelocity(velocity);
    setBottomShooterMotorVelocity(velocity);
  }

  /**
   * 
   * @param velocity
   */
  public void setTopShooterMotorVelocity(double velocity) {
    topShooterMotor.setControl(shooterMotorVelocity.withVelocity(velocity / 60));
  }

  /**
   * 
   * @param velocity
   */
  public void setBottomShooterMotorVelocity(double velocity) {
    bottomShooterMotor.setControl(shooterMotorVelocity.withVelocity(velocity / 60));
  }

  /**
   * Stops the shooter motors
   * 
   * @return
   */
  public void stopShooterMotor() {
    topShooterMotor.stopMotor();
    bottomShooterMotor.stopMotor();
  }

  /**
   * Gets the velocity of the top motor
   * 
   * @return Motor velocity, in RPM
   */
  public double getTopMotorVelocity() {
    return 60 * topShooterMotor.getVelocity().getValue();
  }

  /**
   * Gets the velocity of the bottom motor
   * 
   * @return Motor velocity, in RPM
   */
  public double getBottomMotorVelocity() {
    return 60 * bottomShooterMotor.getVelocity().getValue();
  }

  public double getTargetRPM() {
   double rpm = 0;


      if (RobotContainer.robotState.equals(RobotStates.SHUTTLE)) {

        rpm = ShooterConstants.SHOOTER_SHUTTLE_RPM;

      } else {
        Optional<DriverStation.Alliance> allianceColor = DriverStation.getAlliance();
        
        if (allianceColor.isPresent()) {
      double distance = allianceColor.get().equals(Alliance.Red) ? Cameras.getRobotToRedSpeaker() 
      : Cameras.getRobotToBlueSpeaker();
      DataLogManager.log("Shooter - Calculated Distance: " + distance);

      if (distance < 1.3) {
        DataLogManager.log("*** Shooter - Distance too short too shoot! ***");
        rpm = 3500;
      } else if (distance > 4) {
        DataLogManager.log("*** Shooter - Distance too long too shoot! ***");
        rpm = 4550;
      } else {
        rpm = 11.319 * distance * distance * distance
        - 48.78 * distance * distance
        + 371.44 * distance
        + 3057.6;
      }

        } else {
          DataLogManager.log("*** Shooter - No Alliance Present ***");
        }
      }
    return rpm;
  }

  @Override
  public void periodic() {

  }

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addDoubleProperty("1. Target Shooter RPM", () -> this.targetShooterRPM, null);
      builder.addDoubleProperty("2. Top Shooter RPM", this::getTopMotorVelocity, null);
      // builder.addDoubleProperty("3. Bottom Shooter RPM", this::getBottomMotorVelocity, null);
  }

}
