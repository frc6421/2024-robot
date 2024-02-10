// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/** Add your docs here. */
public class SwerveModule implements Sendable{
  public static class ModuleConstants{
    public static final String RIO_NAME = "rio";

    public static final double DRIVE_KS = 0.0;
    public static final double DRIVE_KV = 0.0;
    public static final double DRIVE_KP = 0.0;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;

    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 35;
    public static final double SUPPLY_CURRENT_THRESHOLD_AMPS = 60;
    public static final double SUPPLY_TIME_THRESHOLD_SEC = 1.0;
    public static final double STATOR_CURRENT_LIMIT_AMPS = 35;

    public static final double STEER_KS = 0.0;
    public static final double STEER_KV = 0.0;
    public static final double STEER_KP = 0.0;
    public static final double STEER_KI = 0.0;
    public static final double STEER_KD = 0.0;

    public static final double MAX_VOLTAGE = 10;
    public static final double WHEEL_DIAMETER_IN = 3.82;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_IN * Math.PI);
    public static final double GEAR_RATIO_MOTOR_TO_WHEEL = 6.75;
    public static final double STEER_GEAR_RATIO = 150.0 / 7.0;
    public static final double DUTY_CYCLE_DEADBAND = 0.045;
    //TODO Verify actual velocity
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.4;
    public static final double TIMEOUT_mS = 50;
  }
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;

  private final CANcoder steerEncoder;

  private final String moduleName;
  private final double rotationOffset; 

  private final TalonFXConfiguration driveMotorConfig;

  private final TalonFXConfiguration steerMotorConfig;

  private final CANcoderConfiguration steerEncoderConfig;

  private final DutyCycleOut driveDutyCycle;
  private final PositionVoltage steerPosition;
  private final VelocityDutyCycle driveVelocity;

  /** Creates a new ModuleSubsystem. */
  public SwerveModule(int driveMotorID, int steerMotorID, int steerEncoderID, double rotationOffset, String name) {
    moduleName = name;
    this.rotationOffset = rotationOffset;

    driveMotor = new TalonFX(driveMotorID, ModuleConstants.RIO_NAME);
    driveMotorConfig = new TalonFXConfiguration();
    driveMotor.getConfigurator().apply(driveMotorConfig, ModuleConstants.TIMEOUT_mS);

    driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    driveMotorConfig.MotorOutput.DutyCycleNeutralDeadband = ModuleConstants.DUTY_CYCLE_DEADBAND;
    driveMotorConfig.Voltage.PeakForwardVoltage = ModuleConstants.MAX_VOLTAGE;
    driveMotorConfig.Voltage.PeakReverseVoltage = - 1.0 * ModuleConstants.MAX_VOLTAGE;
    
    // For velocity control
    driveMotorConfig.Slot0.kS = ModuleConstants.DRIVE_KS;
    driveMotorConfig.Slot0.kV = ModuleConstants.DRIVE_KV;
    driveMotorConfig.Slot0.kP = ModuleConstants.DRIVE_KP;
    driveMotorConfig.Slot0.kI = ModuleConstants.DRIVE_KI;
    driveMotorConfig.Slot0.kD = ModuleConstants.DRIVE_KD;
    
    driveMotorConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.SUPPLY_CURRENT_LIMIT_AMPS;
    driveMotorConfig.CurrentLimits.SupplyCurrentThreshold = ModuleConstants.SUPPLY_CURRENT_THRESHOLD_AMPS;
    driveMotorConfig.CurrentLimits.SupplyTimeThreshold = ModuleConstants.SUPPLY_TIME_THRESHOLD_SEC;
    driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveMotorConfig.CurrentLimits.StatorCurrentLimit = ModuleConstants.STATOR_CURRENT_LIMIT_AMPS;
    driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    driveMotorConfig.Feedback.SensorToMechanismRatio = ModuleConstants.GEAR_RATIO_MOTOR_TO_WHEEL;

    driveMotor.getConfigurator().apply(driveMotorConfig, ModuleConstants.TIMEOUT_mS);

    steerEncoder = new CANcoder(steerEncoderID, ModuleConstants.RIO_NAME);

    steerEncoderConfig = new CANcoderConfiguration();
    steerEncoder.getConfigurator().apply(steerEncoderConfig, ModuleConstants.TIMEOUT_mS);

    steerEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    steerEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    steerEncoderConfig.MagnetSensor.MagnetOffset = rotationOffset;

    steerMotor = new TalonFX(steerMotorID, ModuleConstants.RIO_NAME);
    steerMotorConfig = new TalonFXConfiguration();
    steerMotor.getConfigurator().apply(steerMotorConfig, ModuleConstants.TIMEOUT_mS);

    steerMotorConfig.Voltage.PeakForwardVoltage = ModuleConstants.MAX_VOLTAGE;
    steerMotorConfig.Voltage.PeakReverseVoltage = - 1.0 * ModuleConstants.MAX_VOLTAGE;

    steerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    steerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    steerMotorConfig.Feedback.SensorToMechanismRatio = ModuleConstants.STEER_GEAR_RATIO;

    steerMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

    // For position control 
    steerMotorConfig.Slot0.kS = ModuleConstants.STEER_KS;
    steerMotorConfig.Slot0.kV = ModuleConstants.STEER_KV;
    steerMotorConfig.Slot0.kP = ModuleConstants.STEER_KP;
    steerMotorConfig.Slot0.kI = ModuleConstants.STEER_KI;
    steerMotorConfig.Slot0.kD = ModuleConstants.STEER_KD;

    steerMotorConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.SUPPLY_CURRENT_LIMIT_AMPS;
    steerMotorConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    steerMotorConfig.CurrentLimits.SupplyTimeThreshold = 1.0;
    steerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    steerMotorConfig.CurrentLimits.StatorCurrentLimit = 35;
    steerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    steerMotor.getConfigurator().apply(steerMotorConfig, ModuleConstants.TIMEOUT_mS);
    
    Timer.delay(1.0);
    setSteerMotorToAbsolute();

    driveDutyCycle = new DutyCycleOut(0);
    steerPosition = new PositionVoltage(0);
    driveVelocity = new VelocityDutyCycle(0);

    Shuffleboard.getTab(moduleName).add(this);
  }

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }

  // DRIVE MOTOR METHODS \\

  public double getDriveMotorDutyCycle(){
    return driveMotor.getDutyCycle().refresh().getValue();
  }

  /**
   * Returns the drive motor velocity using encoder counts
   * Not currently used
   * 
   * @return drive motor velocity in meters per second
   */
  public double getDriveMotorVelocity() {
    return driveMotor.getVelocity().refresh().getValue() * ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;
  }

  /**
   * Returns the motor voltage applied to the drive motor in volts
   * Not currently used
   * 
   * @return applied motor voltage in volts
   */
  public double getDriveMotorVoltage() {
    return driveMotor.getMotorVoltage().refresh().getValue();
  }

  /**
   * Returns the drive motor distance using calculation for distance per encoder
   * count
   * 
   * @return drive motor distance in meters
   */
  public double getDriveMotorDistance() {
    return driveMotor.getPosition().refresh().getValue() * ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;
  }

  // STEER MOTOR METHODS \\ 

  /**
   * Sets the of the steer motor encoder to the value of the CANcoder
   * 
   */
  public void setSteerMotorToAbsolute() {
    double currentRotations = steerEncoder.getAbsolutePosition().refresh().getValue();
    double absolutePosition = currentRotations - rotationOffset;
    steerMotor.setPosition(absolutePosition);
  }

  /**
   * Gets the steer motor's current position in rotations
   * @return steer motor's position in rotations
   */
  public double getSteerMotorRotations() {
    return steerMotor.getPosition().refresh().getValue();
  }

  public double getSteerMotorDegrees(){
    return getSteerMotorRotations() * 360;
  }

  // CANCODER METHODS \\

  public double getCANcoderAngle() {
    return steerEncoder.getAbsolutePosition().refresh().getValue() * 180;
  }
  /**
   * Returns the angle measured on the CANcoder (steering encoder)
   * 
   * @return wheel angle in radians
   */
  public double getCANcoderRadians() {
    return Math.toRadians(getCANcoderAngle());
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
  public void setTeleopDesiredState(SwerveModuleState desiredState) {
    // Optimize the desired state to avoid spinning modules more than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(Math.toRadians(getSteerMotorDegrees())));

    // Calculate percent of max drive velocity
    driveDutyCycle.Output = (state.speedMetersPerSecond / ModuleConstants.MAX_VELOCITY_METERS_PER_SECOND);

    // Calculate steer motor position in rotations
    steerPosition.Position = state.angle.getDegrees() / 360;

    // if(driverController.povLeft().getAsBoolean()){ 
    //   steerPositionOutput =  (state.angle.getDegrees() + 90 - state.angle.getDegrees() % 360) * DriveConstants.STEER_MOTOR_ENCODER_COUNTS_PER_DEGREE;
    // }

    // Apply PID outputs
    //driveMotor.set(ControlMode.PercentOutput, driveOutput);
    driveMotor.setControl(driveDutyCycle);
    steerMotor.setControl(steerPosition);
    
  }

  /**
   * Optimizes the swerve module outputs and applies the drive percent output and steer position
   * Closed loop output
   * 
   * @param desiredState
   */
  public void setAutoDesiredState(SwerveModuleState desiredState) {
    // Optimize the desired state to avoid spinning modules more than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(Math.toRadians(getSteerMotorDegrees())));

    // Calculate percent of max drive velocity
    driveVelocity.Velocity = state.speedMetersPerSecond;

    // Calculate steer motor output
    steerPosition.Position = state.angle.getDegrees() / 360;

    // Apply PID outputs
    //driveMotor.set(ControlMode.Velocity, driveOutput, DemandType.ArbitraryFeedForward, feedforward.calculate(state.speedMetersPerSecond));
    //steerMotor.set(ControlMode.Position, steerPositionOutput);

    driveMotor.setControl(driveVelocity);
    steerMotor.setControl(steerPosition);
  }

  public void resetEncoders() {
    //TODO: determine what to do for drive motor reset
    driveMotor.setPosition(0, ModuleConstants.TIMEOUT_mS);
    steerEncoder.setPosition(0, ModuleConstants.TIMEOUT_mS);
  }

  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("SwerveModule");

    builder.addDoubleProperty("Drive Motor Velocity", this::getDriveMotorVelocity, null);
    builder.addDoubleProperty("Steer Motor Angle", this::getSteerMotorDegrees, null);
    builder.addDoubleProperty("CANCoder Angle Degrees", this::getCANcoderAngle, null);
    builder.addDoubleProperty("Drive Motor Output", this::getDriveMotorDutyCycle, null);
    builder.addDoubleProperty("Drive Motor Voltage", this::getDriveMotorVoltage, null);
    builder.addDoubleProperty("Drive Motor Distance Meters", this::getDriveMotorDistance, null);
  }
}
