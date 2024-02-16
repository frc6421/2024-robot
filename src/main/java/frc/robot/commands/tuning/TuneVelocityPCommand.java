// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tuning;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveConstants;

public class TuneVelocityPCommand extends Command {
  private final DriveSubsystem driveSubsystem;

  private Slot0Configs slot0Configs;

  private ChassisSpeeds setChassisSpeeds;

  private final SwerveRequest.PointWheelsAt zeroWheelsRequest;
  private final SwerveRequest.ApplyChassisSpeeds driveVelocityRequest;
  private final SwerveRequest.ApplyChassisSpeeds stopRobotRequest;

  private double velocity;
  private double velocitykS;
  private double velocitykV;
  private double velocityP;
  private double finalVelocity;

  /** Creates a new TuneVelocityPCommand. */
  public TuneVelocityPCommand(DriveSubsystem drive) {
    driveSubsystem = drive;

    // Config for PID. Used to reset the kP Config
    slot0Configs = new Slot0Configs();

    zeroWheelsRequest = new SwerveRequest.PointWheelsAt();
    driveVelocityRequest = new SwerveRequest.ApplyChassisSpeeds().withDriveRequestType(DriveRequestType.Velocity);
    stopRobotRequest = new SwerveRequest.ApplyChassisSpeeds();

    velocity = 0;
    velocityP = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    Shuffleboard.getTab("5: Velocity P").add(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Update the Slot0Config to the values set in the ShuffleBOard tab
    for (int i = 0; i < 4; ++i) {
      driveSubsystem.getModule(i).getDriveMotor().getConfigurator().refresh(slot0Configs);
      slot0Configs.withKS(velocitykS);
      slot0Configs.withKV(velocitykV);
      slot0Configs.withKP(velocityP);
      driveSubsystem.getModule(i).getDriveMotor().getConfigurator().apply(slot0Configs);
    }

    // Set wheels to face forward.
    driveSubsystem.setControl(zeroWheelsRequest.withModuleDirection(new Rotation2d()));
    // Reset odometry
    driveSubsystem.tareEverything();
    // Set chassis speed based on the velocity set in Shuffleboard
    setChassisSpeeds = new ChassisSpeeds(velocity, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setControl(driveVelocityRequest.withSpeeds(setChassisSpeeds));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Get the final velocity before stop the motors.
    finalVelocity = driveSubsystem.getSwerveDriveKinematics()
        .toChassisSpeeds(driveSubsystem.getState().ModuleStates).vxMetersPerSecond;

    // Stop the robot.
    driveSubsystem.setControl(stopRobotRequest.withSpeeds(new ChassisSpeeds(0, 0, 0)));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stop the command when the robot has travelled at least 3.5 meters.
    return Math.abs(driveSubsystem.getCurrentPose2d().getX()) > 4.5;
  }

  /**
   * The kS value to set for the velocity PID config
   * 
   * @param velocitykS
   */
  private void setVelocitykS(double velocitykS) {
    this.velocitykS = velocitykS;
  }

  /**
   * The kV value to set for the velocity PID config
   * 
   * @param velocitykV
   */
  private void setVelocitykV(double velocitykV) {
    this.velocitykV = velocitykV;
  }

  /**
   * The p value to set for the velocity PID config.
   * 
   * @param velocityP in units of voltage / rpm
   */
  private void setVelocityP(double velocityP) {
    this.velocityP = velocityP;
  }

  /**
   * Set velocity to run the robot at
   * 
   * @param velocity x velocity of the robot (-
   *                 {@link DriveSubsystem#SPEED_AT_12_VOLTS_METERS_PER_SEC} to
   *                 {@link DriveSubsystem#SPEED_AT_12_VOLTS_METERS_PER_SEC})
   */
  private void setVelocity(double velocity) {
    this.velocity = MathUtil.clamp(velocity, -1 * DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC,
        DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Velocity", () -> velocity, this::setVelocity);
    builder.addDoubleProperty("Velocity kS", () -> velocitykS, this::setVelocitykS);
    builder.addDoubleProperty("Velocity kV", () -> velocitykV, this::setVelocitykV);
    builder.addDoubleProperty("Velocity P", () -> velocityP, this::setVelocityP);
    builder.addDoubleProperty("Final Velocity", () -> finalVelocity, null);
    builder.addDoubleProperty("Current Velocity", () -> driveSubsystem.getSwerveDriveKinematics()
        .toChassisSpeeds(driveSubsystem.getState().ModuleStates).vxMetersPerSecond, null);
  }

}
