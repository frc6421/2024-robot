// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Cameras;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotStates;
import frc.robot.Constants.VisionConstants;

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
  private TalonFX topShooterMotor;
  private TalonFX bottomShooterMotor;

  private TalonFXConfiguration topShooterConfig;
  private TalonFXConfiguration bottomShooterConfig;

  private VelocityVoltage shooterMotorVelocity;

  public double targetShooterRPM;

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

  }

  /**
   * Runs the motors at a desired velocity
   * 
   * @param velocity the speed of which to run the motors, in RPM
   */
  public void setShooterMotorVelocity(double velocity) {
    shooterMotorVelocity.withVelocity(velocity / 60);
    topShooterMotor.setControl(shooterMotorVelocity);
    bottomShooterMotor.setControl(shooterMotorVelocity);
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
    Optional<DriverStation.Alliance> allianceColor = DriverStation.getAlliance();

    if (allianceColor.isPresent()) {

      if (RobotContainer.robotState.equals(RobotStates.SHUTTLE)) {

        return ShooterConstants.SHOOTER_SHUTTLE_RPM;

      } else {

        targetTagID = allianceColor.get().equals(Alliance.Red) ? 4 : 7;

        double pitchAngle = Cameras.getPitch(Cameras.speakerCamera, targetTagID);

        if (pitchAngle > VisionConstants.SPEAKER_PITCH_ARRAY[0]) {

          if(DriverStation.isAutonomous()) {

            return 4670;
            
          } else {

            return VisionConstants.SHOOTER_RPM_ARRAY[0];

          }

        } else if (pitchAngle < VisionConstants.SPEAKER_PITCH_ARRAY[VisionConstants.SPEAKER_PITCH_ARRAY.length - 1]) {

          return VisionConstants.SHOOTER_RPM_ARRAY[VisionConstants.SPEAKER_PITCH_ARRAY.length - 1];

        } else {

          if(targetTagID == 4) {

            return (0.007 * Math.pow(pitchAngle, 4) - 0.1335 * Math.pow(pitchAngle, 3) + 0.3034 * Math.pow(pitchAngle, 2) - 20.564 * pitchAngle + 3824.6);

          } else {

            return (0.0054 * Math.pow(pitchAngle, 4) - 0.1195 * Math.pow(pitchAngle, 3) + 0.7534 * Math.pow(pitchAngle, 2) - 23.165 * pitchAngle + 3812.6);
          }

        }

      }

    } else {

      return VisionConstants.SHOOTER_RPM_ARRAY[0];

    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("bottom RPM", getBottomMotorVelocity());
    // SmartDashboard.putNumber("top RPM", getTopMotorVelocity());

    /*
     * Optional<DriverStation.Alliance> allianceColor = DriverStation.getAlliance();
     * 
     * if (allianceColor.isPresent()) {
     * 
     * targetTagID = allianceColor.get().equals(Alliance.Red) ? 4 : 7;
     * 
     * }
     * 
     * if (Cameras.isTarget(Cameras.speakerCamera)) {
     * 
     * // Check distance from the target using camera pitch
     * for (int i = 0; i < VisionConstants.SPEAKER_PITCH_ARRAY.length; i++) {
     * 
     * if(Cameras.getPitch(Cameras.speakerCamera, targetTagID) >=
     * VisionConstants.SPEAKER_PITCH_ARRAY[i]) {
     * 
     * targetShooterRPM = VisionConstants.SHOOTER_RPM_ARRAY[i];
     * 
     * break;
     * 
     * }
     * 
     * }
     * 
     * }
     * 
     */

    //SmartDashboard.putNumber("Target RPM", getTargetRPM());

  }

}
