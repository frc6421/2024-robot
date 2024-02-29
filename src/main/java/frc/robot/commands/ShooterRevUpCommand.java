// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterConstants;

public class ShooterRevUpCommand extends Command {

  private final ShooterSubsystem shooterSubsystem;

  private Timer time;

  /** Creates a new ShooterRevUp. */
  public ShooterRevUpCommand(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(shooterSubsystem);

    time = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set shooter to slower idle speed
    shooterSubsystem.setTopShooterMotorVelocity(ShooterConstants.SHOOTER_IDLE_RPM);
    shooterSubsystem.setBottomShooterMotorVelocity(ShooterConstants.SHOOTER_IDLE_RPM);

    time.reset();
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    time.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO determine if we need a timer condition
    return ((shooterSubsystem.getBottomMotorVelocity() >= ShooterConstants.SHOOTER_IDLE_RPM - 50 && 
    shooterSubsystem.getTopMotorVelocity() >= ShooterConstants.SHOOTER_IDLE_RPM - 50));
  }
}