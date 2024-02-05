// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tuning;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
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
  private double velocityP;

  /** Creates a new TuneVelocityPCommand. */
  public TuneVelocityPCommand(DriveSubsystem drive) {
    driveSubsystem = drive;

    slot0Configs = new Slot0Configs();

    zeroWheelsRequest = new SwerveRequest.PointWheelsAt();
    driveVelocityRequest = new SwerveRequest.ApplyChassisSpeeds();
    stopRobotRequest = new SwerveRequest.ApplyChassisSpeeds();

    velocity = 0;
    velocityP = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    for (int i = 0; i < 4; ++i) {
      driveSubsystem.getModule(i).getDriveMotor().getConfigurator().refresh(slot0Configs);
      slot0Configs.withKP(velocityP);
      driveSubsystem.getModule(i).getDriveMotor().getConfigurator().apply(slot0Configs);
    }

    // Reset odometry
    driveSubsystem.seedFieldRelative();
    // Set wheels to face forward.
    driveSubsystem.setControl(zeroWheelsRequest);
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
    for (int i = 0; i < 4; ++i) {
      driveSubsystem.getModule(i).getDriveMotor().getConfigurator().refresh(slot0Configs);
      System.out.println("Module " + i + " Slot 0 Config");
      System.out.println(slot0Configs.toString());
    }

    driveSubsystem.setControl(stopRobotRequest);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stop the command when the robot has travelled at least 3.5 meters.
    return Math.abs(driveSubsystem.getState().Pose.getX()) > 3.5;
  }

  /**
   * The p value to set for the velocity PID.
   * 
   * @param velocityP in units of voltage / rpm (?)
   */
  public void setVelocityP(double velocityP) {
    this.velocityP = velocityP;
  }

  /**
   * Set velocity to run the robot at
   * 
   * @param velocity x velocity of the robot (-
   *                 {@link DriveSubsystem#SPEED_AT_12_VOLTS_METERS_PER_SEC} to
   *                 {@link DriveSubsystem#SPEED_AT_12_VOLTS_METERS_PER_SEC})
   */
  public void setVelocity(double velocity) {
    this.velocity = MathUtil.clamp(velocity, -1 * DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC,
        DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Velocity", () -> velocity, this::setVelocity);
    builder.addDoubleProperty("Velocity P", () -> velocityP, this::setVelocityP);
  }

}
