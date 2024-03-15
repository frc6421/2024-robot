// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterRevUpCommand extends Command {

  private final ShooterSubsystem shooterSubsystem;
  private double shooterVelocity;

  double topShooterOffset = -100;
  double velocityErrorRange = 100;

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

    shooterVelocity = shooterSubsystem.getTargetRPM();

    // set shooter to target speed
    shooterSubsystem.setTopShooterMotorVelocity(shooterVelocity);
    shooterSubsystem.setBottomShooterMotorVelocity(shooterVelocity);

    time.reset();
    time.start();

    System.out.println("Target Shooter Velocity: " + shooterVelocity + " @ " + Timer.getMatchTime());
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
    return ((shooterSubsystem.getBottomMotorVelocity() >= shooterVelocity - velocityErrorRange && 
    shooterSubsystem.getTopMotorVelocity() >= shooterVelocity + topShooterOffset - velocityErrorRange))
    || time.hasElapsed(0.7);
  }
}