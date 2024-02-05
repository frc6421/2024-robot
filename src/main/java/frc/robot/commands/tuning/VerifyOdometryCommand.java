// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tuning;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveConstants;

public class VerifyOdometryCommand extends Command {
  private final DriveSubsystem driveSubsystem;

  private final SwerveRequest.PointWheelsAt zeroWheels;

  /**
   * Verify that odometry calcualtion is same as actual distance travelled.
   * </p>
   * If odometry is off by more than a acceptable range for the team, it is
   * suggested that the wheel be physically measured with a caliper and the
   * measured value entered into {@link DriveSubsystem#WHEEL_RADIUS_INCHES}.
   * </p>
   * After the value is changed the command should be run again to verify the
   * odometry is within the acceptable range.
   * 
   * @param drive the drive subsystem used with command.
   */
  public VerifyOdometryCommand(DriveSubsystem drive) {
    driveSubsystem = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    // Swerve request to make sure the wheels point in the x direction
    zeroWheels = new SwerveRequest.PointWheelsAt();

    // Add command to shuffleboard.
    Shuffleboard.getTab("1: Verify Odometry").add(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Point wheels forward
    driveSubsystem.setControl(zeroWheels);
    // Reset robot Pose
    driveSubsystem.seedFieldRelative();
    // Set drive motors to coast
    driveSubsystem.configNeutralMode(NeutralModeValue.Coast);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Do nothing. Manually move the robot to distances.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Final robot distance (meters) :" + driveSubsystem.getState().Pose.getX());
    for (int i = 0; i < 4; ++i) {
      System.out.println(
          "Module " + i + " distance (meters): " + driveSubsystem.getModule(i).getDriveMotor().getPosition().getValue()
              / DriveConstants.DRIVE_ROTATIONS_PER_METER);
    }
    // Set drive motors to brake.
    driveSubsystem.configNeutralMode(NeutralModeValue.Brake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Robot Distance (meters)", () -> driveSubsystem.getState().Pose.getX(), null);
    builder.addDoubleProperty("Module 0 Distance (meters)",
        () -> driveSubsystem.getModule(0).getDriveMotor().getPosition().getValue()
            / DriveConstants.DRIVE_ROTATIONS_PER_METER,
        null);
    builder.addDoubleProperty("Module 1 Distance (meters)",
        () -> driveSubsystem.getModule(1).getDriveMotor().getPosition().getValue()
            / DriveConstants.DRIVE_ROTATIONS_PER_METER,
        null);
    builder.addDoubleProperty("Module 2 Distance (meters)",
        () -> driveSubsystem.getModule(2).getDriveMotor().getPosition().getValue()
            / DriveConstants.DRIVE_ROTATIONS_PER_METER,
        null);
    builder.addDoubleProperty("Module 3 Distance (meters)",
        () -> driveSubsystem.getModule(3).getDriveMotor().getPosition().getValue()
            / DriveConstants.DRIVE_ROTATIONS_PER_METER,
        null);
  }
}
