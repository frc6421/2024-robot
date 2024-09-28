// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem.AngleConstants;

public class ShooterTuningCommand extends Command {
  ShooterSubsystem shooterSubsystem;
  ShooterAngleSubsystem shooterAngleSubsystem;
  double velocity = 0;
  double angle = AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES;
  /** Creates a new ShooterTuningCommand. */
  public ShooterTuningCommand(ShooterAngleSubsystem shooterAngleSubsystem, ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.shooterAngleSubsystem = shooterAngleSubsystem;

    addRequirements(shooterSubsystem, shooterAngleSubsystem);

    Shuffleboard.getTab("Shooter Tuning").add(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setShooterMotorVelocity(velocity);
    shooterAngleSubsystem.setAngle(() -> angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooterMotor();
    shooterAngleSubsystem.setAngle(() -> AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

   @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Set Shooter RPM", () -> velocity, this::setShooterVelocity);
    builder.addDoubleProperty("Set Shooter Angle", () -> angle, this::setShooterAngle);
  }

  private void setShooterVelocity(double velocity) {
   this.velocity = velocity; 
  }

  private void setShooterAngle(double angle) {
    this.angle = angle;
  }
}