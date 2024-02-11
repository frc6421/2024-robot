// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveConstants;

public class DriveCommand extends Command {
  private DriveSubsystem driveSubsystem;

  private double percentDeadband = 0.1;

  private double maxSpeed = DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC;
  private double maxAngularRate = 2 * Math.PI;

  private final CommandXboxController driverController;


  private final SwerveRequest.FieldCentric driveRequest; 

  private final SlewRateLimiter xDriveSlew;
  private final SlewRateLimiter yDriveSlew;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem drive, CommandXboxController controller) {

    driveSubsystem = drive;
    driverController = controller;
   

    driveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(maxSpeed * percentDeadband).withRotationalDeadband(maxAngularRate * percentDeadband) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    xDriveSlew = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_RATE);
    yDriveSlew = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_RATE);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed;
    double ySpeed;

    xSpeed = xDriveSlew.calculate(-driverController.getLeftY() * DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC);
    ySpeed = yDriveSlew.calculate(-driverController.getLeftX() * DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC);

    driveSubsystem.setControl(
        driveRequest.withVelocityX(xSpeed)
        .withVelocityY(ySpeed)
        .withRotationalRate(-driverController.getRightX() * DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

}