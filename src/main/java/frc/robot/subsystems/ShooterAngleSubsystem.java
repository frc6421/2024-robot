// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Cameras;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotStates;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.VisionConstants;

import com.revrobotics.RelativeEncoder;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import javax.swing.text.html.Option;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;

public class ShooterAngleSubsystem extends SubsystemBase {
  public static class AngleConstants {
    public static final int ANGLE_CAN_ID = 32;
    public static final int CURRENT_LIMIT = 60;

    // PID values
    public static final double ANGLE_KP = 0.16;
    public static final double ANGLE_KI = 0;
    public static final double ANGLE_KD = 0;

    // Limits
    public static final float MAXIMIMUM_SOFT_LIMIT_DEGREES = 52;
    public static final float MINIMUM_SOFT_LIMIT_DEGREES = -25;

    // Encoder conversions
    public static final int GEAR_RATIO = 180;
    public static final double DEGREES_PER_MOTOR_ROTATION = (360.0 / AngleConstants.GEAR_RATIO);

    public static final double SHOOTER_PIVOT_SHUTTLE_ANGLE = 40.0;

  }

  // Creating the object for the motor and encoder
  private CANSparkMax angleMotor;

  private SparkPIDController angleMotorPID;

  private static RelativeEncoder angleEncoder;

  private double positionMinOutput;
  private double positionMaxOutput;

  public double targetShooterPivotAngle = AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES;

  private int targetTagID = 0;

  /** Creates a new AngledShooterSubsystem. */
  public ShooterAngleSubsystem() {
    // Setting CAN ID and the type
    angleMotor = new CANSparkMax(AngleConstants.ANGLE_CAN_ID, MotorType.kBrushless);

    // Reseting to defaults
    angleMotor.restoreFactoryDefaults();

    // Set the idle mode of the motor
    angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Inverts
    angleMotor.setInverted(true);

    // Current limit
    angleMotor.setSmartCurrentLimit(AngleConstants.CURRENT_LIMIT);

    // Clearing faults
    angleMotor.clearFaults();

    // Setting encoder with the conversion factor of the gearbox
    angleEncoder = angleMotor.getEncoder();
    angleEncoder.setPositionConversionFactor(AngleConstants.DEGREES_PER_MOTOR_ROTATION);

    // Reseting the encoder to 0
    angleEncoder.setPosition(AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES);

    // How much of the output range one wants to use
    positionMinOutput = -1;
    positionMaxOutput = 1;

    // Getting the motor for PID
    angleMotorPID = angleMotor.getPIDController();

    // Setting the feedback device of the motor to the built-in encoder
    angleMotorPID.setFeedbackDevice(angleEncoder);

    // Setting motor PID values
    angleMotorPID.setP(AngleConstants.ANGLE_KP, 0);
    angleMotorPID.setI(AngleConstants.ANGLE_KI, 0);
    angleMotorPID.setD(AngleConstants.ANGLE_KD, 0);

    // Setting and enabling the soft limits of the motors
    angleMotor.setSoftLimit(SoftLimitDirection.kForward, AngleConstants.MAXIMIMUM_SOFT_LIMIT_DEGREES);
    angleMotor.setSoftLimit(SoftLimitDirection.kReverse, AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES);
    angleMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    angleMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    angleMotorPID.setOutputRange(positionMinOutput, positionMaxOutput, 0);

    // Shuffeboard tab
    Shuffleboard.getTab("Shooter Angle").add(this);
  }

  /**
   * Sets the output of the angle motor to go to a certain angle
   * 
   * @param angle The angle of which to set the motor to
   */
  public void setAngle(DoubleSupplier angle) {
    targetShooterPivotAngle = angle.getAsDouble();
    angleMotorPID.setReference(angle.getAsDouble(), CANSparkMax.ControlType.kPosition, 0, 0,
        SparkPIDController.ArbFFUnits.kVoltage);
  }

  /**
   * Gets the position of the encoder to compare to the actual value
   * 
   * @return Arm position, in degrees
   */
  public double getAngleEncoderPosition() {
    return angleEncoder.getPosition();
  }

  public double getTargetAngle() {

   double angle = AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES;

   if (RobotContainer.robotState.equals(RobotStates.SHUTTLE)) {
      angle = AngleConstants.SHOOTER_PIVOT_SHUTTLE_ANGLE;
   } else {
    Optional<DriverStation.Alliance> allianceColor = DriverStation.getAlliance();
    if (allianceColor.isPresent()) {
      double distance = allianceColor.get().equals(Alliance.Red) ? Cameras.getRobotToRedSpeaker() 
      : Cameras.getRobotToBlueSpeaker();
      DataLogManager.log("Shooter Angle - Calculated Distance: " + distance);

      if (distance < 1.3) {
        DataLogManager.log("*** ShooterAngle - Distance too short too shoot! ***");
        angle = TrajectoryConstants.DEGREE_AT_SUBWOOFER;
      } else if (distance > 5) {
        DataLogManager.log("*** ShooterAngle - Distance too long too shoot! ***");
        angle = TrajectoryConstants.DEGREE_AT_LONGEST;
      }

     else {
      angle = 2.4492 * distance * distance
       - 23.307 * distance
       + 78.295
       + 0.5;
    }

   } else {
    DataLogManager.log("*** Shooter Angle - No Alliance Present! ***");
   }
  }
  return angle;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);

      builder.addDoubleProperty("4. Target Angle", () -> targetShooterPivotAngle, null);
      builder.addDoubleProperty("5. Actual Angle", this::getAngleEncoderPosition, null);
  }

}
