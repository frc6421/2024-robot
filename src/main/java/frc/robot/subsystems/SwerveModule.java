// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class SwerveModule {
  public static class ModuleConstants{
    public static final String RIO_NAME = "rio";

    public static final double DRIVE_KS = 0.0;
    public static final double DRIVE_KV = 0.0;
    public static final double DRIVE_KP = 0.0;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;

    public static final double STEER_KS = 0.0;
    public static final double STEER_KV = 0.0;
    public static final double STEER_KP = 0.0;
    public static final double STEER_KI = 0.0;
    public static final double STEER_KD = 0.0;
    public static final double MAX_VOLTAGE = 10;
    public static final int COUNTS_PER_ROTATION = 0;
  }
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;

  private final CoreCANcoder steerEncoder;

  private final String moduleName;

  private final TalonFXConfiguration driveMotorConfig;

  private final TalonFXConfiguration steerMotorConfig;
  /** Creates a new ModuleSubsystem. */
  public SwerveModule(int driveMotorID, int steerMotorID, int steerEncoderID, double rotationOffset, String name) {
    moduleName = name;

    driveMotor = new TalonFX(driveMotorID, ModuleConstants.RIO_NAME);
    driveMotorConfig = new TalonFXConfiguration();
    driveMotor.getConfigurator().apply(driveMotorConfig);

    driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    driveMotorConfig.Voltage.PeakForwardVoltage = ModuleConstants.MAX_VOLTAGE;
    driveMotorConfig.Voltage.PeakReverseVoltage = - 1.0 * ModuleConstants.MAX_VOLTAGE;
    
    // For velocity control
    driveMotorConfig.Slot0.kS = ModuleConstants.DRIVE_KS;
    driveMotorConfig.Slot0.kV = ModuleConstants.DRIVE_KV;
    driveMotorConfig.Slot0.kP = ModuleConstants.DRIVE_KP;
    driveMotorConfig.Slot0.kI = ModuleConstants.DRIVE_KI;
    driveMotorConfig.Slot0.kD = ModuleConstants.DRIVE_KD;
    
    // TODO Verify if there is a deadband config.
    //driveMotor.configNeutralDeadband(ModuleConstants.PERCENT_DEADBAND);
    driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 35;
    driveMotorConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    driveMotorConfig.CurrentLimits.SupplyTimeThreshold = 1.0;
    driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveMotorConfig.CurrentLimits.StatorCurrentLimit = 35;
    driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    driveMotor.getConfigurator().apply(driveMotorConfig);

    steerEncoder = new CoreCANcoder(steerEncoderID, ModuleConstants.RIO_NAME);
    steerEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    steerEncoder.configSensorDirection(false); // Counter Clockwise
    steerEncoder.configMagnetOffrotation();
    steerEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    steerMotor = new TalonFX(steerMotorID, ModuleConstants.RIO_NAME);
    steerMotorConfig = new TalonFXConfiguration();
    steerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    steerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // unknown solution
    //steerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    //steerMotorConfig.Feedback.sensor = 

    // For position control 
    steerMotorConfig.Slot0.kS = ModuleConstants.STEER_KS;
    steerMotorConfig.Slot0.kV = ModuleConstants.STEER_KV;
    steerMotorConfig.Slot0.kP = ModuleConstants.STEER_KP;
    steerMotorConfig.Slot0.kI = ModuleConstants.STEER_KI;
    steerMotorConfig.Slot0.kD = ModuleConstants.STEER_KD;

    // Unsure how to fix this line
    //steerMotor.configAllowableClosedloopError(0, 0.5 * DriveConstants.STEER_MOTOR_ENCODER_COUNTS_PER_DEGREE);
    // Same deadband error
    //steerMotor.configNeutralDeadband(ModuleConstants.PERCENT_DEADBAND);

    steerMotorConfig.CurrentLimits.SupplyCurrentLimit = 35;
    steerMotorConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    steerMotorConfig.CurrentLimits.SupplyTimeThreshold = 1.0;
    steerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    steerMotorConfig.CurrentLimits.StatorCurrentLimit = 35;
    steerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    
    Timer.delay(1.0);
    setSteerMotorToAbsolute();
  }

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }

  // DRIVE MOTOR METHODS \\

  /**
   * Returns the drive motor velocity using encoder counts
   * Not currently used
   * 
   * @return drive motor velocity in meters per second
   */
  public double getDriveMotorVelocity() {
    return ((driveMotor.getSelectedSensorVelocity() / DriveConstants.GEAR_RATIO_MOTOR_TO_WHEEL) *
        (10.0 / ModuleConstants.COUNTS_PER_ROTATION) * DriveConstants.WHEEL_CIRCUMFERENCE);
  }

  /**
   * Returns the motor voltage applied to the drive motor in volts
   * Not currently used
   * 
   * @return applied motor voltage in volts
   */
  public double getDriveMotorVoltage() {
    return driveMotor.getMotorOutputVoltage();
  }

  /**
   * Returns the drive motor distance using calculation for distance per encoder
   * count
   * 
   * @return drive motor distance in meters
   */
  public double getDriveMotorDistance() {
    return driveMotor.getSelectedSensorPosition() * DriveConstants.DISTANCE_PER_ENCODER_COUNT;
  }

  /**
   * Converts the drive encoder velocity from counts per 100 ms to meters per
   * second
   * 
   * @return drive encoder velocity in meters per second
   */
  public double getDriveMotorEncoderVelocity() {
    return driveMotor.getSelectedSensorVelocity() * 10 * DriveConstants.DISTANCE_PER_ENCODER_COUNT;
  }

  // STEER MOTOR METHODS \\

  /**
   * Sets the of the steer motor encoder to the value of the CANcoder
   * 
   */
  public void setSteerMotorToAbsolute() {
    double currentAngle = steerEncoder.getAbsolutePosition();
    double absolutePosition = currentAngle * DriveConstants.STEER_MOTOR_ENCODER_COUNTS_PER_DEGREE;
    steerMotor.setSelectedSensorPosition(absolutePosition);
  }

  /**
   * Gets the steer motor's current angle in degrees
   * @return steer motor's angle in degrees
   */
  public double getSteerMotorEncoderAngle() {
    return steerMotor.getSelectedSensorPosition() / ModuleConstants.STEER_MOTOR_ENCODER_COUNTS_PER_DEGREE;
  }

  // CANCODER METHODS \\

  /**
   * Returns the angle measured on the CANcoder (steering encoder)
   * 
   * @return wheel angle in radians
   */
  public double getCANcoderRadians() {
    return Math.toRadians(steerEncoder.getAbsolutePosition());
  }

  /**
   * Gets the module position based on distance traveled in meters for drive motor
   * and degrees for steering motor
   * Used for odometry
   * 
   * @return current SwerveModulePosition
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getDriveMotorDistance(), new Rotation2d(getCANcoderRadians()));
  }

  /**
   * Optimizes the swerve module outputs and applies the drive percent output and steer position
   * 
   * @param desiredState
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the desired state to avoid spinning modules more than 90 degrees
    SwerveModuleState state = customOptimize(desiredState, new Rotation2d(Math.toRadians(getSteerMotorEncoderAngle())));

    // Calculate percent of max drive velocity
    double driveOutput = (state.speedMetersPerSecond / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);

    // Calculate steer motor output
    double steerPositionOutput = state.angle.getDegrees() * DriveConstants.STEER_MOTOR_ENCODER_COUNTS_PER_DEGREE;

    // if(driverController.povLeft().getAsBoolean()){ 
    //   steerPositionOutput =  (state.angle.getDegrees() + 90 - state.angle.getDegrees() % 360) * DriveConstants.STEER_MOTOR_ENCODER_COUNTS_PER_DEGREE;
    // }

    // Apply PID outputs
    driveMotor.set(ControlMode.PercentOutput, driveOutput);
    steerMotor.set(ControlMode.Position, steerPositionOutput);
  }

  /**
   * Optimizes the swerve module outputs and applies the drive percent output and steer position
   * Closed loop output
   * 
   * @param desiredState
   */
  public void autoSetDesiredState(SwerveModuleState desiredState) {
    // Optimize the desired state to avoid spinning modules more than 90 degrees
    SwerveModuleState state = customOptimize(desiredState, new Rotation2d(Math.toRadians(getSteerMotorEncoderAngle())));

    // Calculate percent of max drive velocity
    double driveOutput = state.speedMetersPerSecond / DriveConstants.DISTANCE_PER_ENCODER_COUNT / 10;

    // Calculate steer motor output
    double steerPositionOutput = state.angle.getDegrees() * DriveConstants.STEER_MOTOR_ENCODER_COUNTS_PER_DEGREE;

    // Apply PID outputs
    driveMotor.set(ControlMode.Velocity, driveOutput, DemandType.ArbitraryFeedForward, feedforward.calculate(state.speedMetersPerSecond));
    steerMotor.set(ControlMode.Position, steerPositionOutput);
  }


  public void resetEncoders() {
    driveMotorConfig.setSelectedSensorPosition    setSelectedSensorPosition(0);
    steerEncoder.setPosition(0);
  }

  /**
   * From team 364
   * 
   * Minimize the change in heading the desired swerve module state would require
   * by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to
   * include placing
   * in appropriate scope for CTRE onboard control.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState customOptimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if(Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * From team 364
   * 
   * @param initialAngle Current Angle
   * @param targetAngle  Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double initialAngle, double targetAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = initialAngle % 360;
    if (lowerOffset >= 0) {
      lowerBound = initialAngle - lowerOffset;
      upperBound = initialAngle + (360 - lowerOffset);
    } else {
      upperBound = initialAngle - lowerOffset;
      lowerBound = initialAngle - (360 + lowerOffset);
    }
    while (targetAngle < lowerBound) {
      targetAngle += 360;
    }
    while (targetAngle > upperBound) {
      targetAngle -= 360;
    }
    if (targetAngle - initialAngle > 180) {
      targetAngle -= 360;
    } else if (targetAngle - initialAngle < -180) {
      targetAngle += 360;
    }
    return targetAngle;
  }
}
