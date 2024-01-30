// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class TuneVelocityPCommand extends Command {

  private final IntakeSubsystem intakeSubsystem;

  private double velocity;
  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double kV = 0;
  private double P = 0;

  /** Creates a new TuneVelocityPCommand. */
  public TuneVelocityPCommand(IntakeSubsystem intake) {
    intakeSubsystem = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

  }

  private void setVelocity(double value) {
    velocity = value;
  }

  private void setkV(double value) {
    kV = value;
  }

  private void setkP(double value) {
    kP = value;
  }

  private void setP(double value) {
    P = value;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("velocity: ", () -> velocity, this::setVelocity);
    builder.addDoubleProperty("kV", () -> kV, this::setkV);
    builder.addDoubleProperty("kP", () -> kP, this::setkP);
  }
}