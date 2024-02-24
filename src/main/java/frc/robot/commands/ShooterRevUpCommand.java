// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
  public void initialize() 
  {
    shooterSubsystem.setShooterMotorVelocity(ShooterConstants.SHOOTER_SUB_RPM);
    time.reset();
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    time.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((shooterSubsystem.getBottomMotorVelocity() >= ShooterConstants.SHOOTER_SUB_RPM - 50 && 
    shooterSubsystem.getTopMotorVelocity() >= ShooterConstants.SHOOTER_SUB_RPM - 50) || time.get() >= 0.4);
  }
}
