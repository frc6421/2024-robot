// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class TuneShooter extends Command {

  private double topTune[];
  private double bottomTune[];

  private double setVelocity;
  private double finalTopVelocity;
  private double finalBottomVelocity;
  private double finalTopkV;
  private double finalBottomkV;

  private final ShooterSubsystem shooterSubsystem;
  /** Creates a new TuneShooter. */
  public TuneShooter(ShooterSubsystem shooter) {
    topTune = new double[3];
    bottomTune = new double[3];

    shooterSubsystem = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    Shuffleboard.getTab("Shooter Tuning").add(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
    builder.addDoubleProperty("Top kP", null, () -> topShooterConfig.Slot0.kP);
    builder.addDoubleProperty("Top kS", () -> topTune[1], null);
    builder.addDoubleProperty("Top kV", () -> topTune[2], null);
    builder.addDoubleProperty("Bottom kP", () -> bottomTune[0], null);
    builder.addDoubleProperty("Bottom kS", () -> bottomTune[1], null);
    builder.addDoubleProperty("Bottom kV", () -> bottomTune[2], null);
    builder.addDoubleProperty("Set Velocity", () -> setVelocity, null);
    builder.addDoubleProperty("Top Motor Velocity", () -> shooterSubsystem.getTopMotorVelocity(), null);
    builder.addDoubleProperty("Bottom Motor Velocity", () -> shooterSubsystem.getBottomMotorVelocity(), null);
    builder.addDoubleProperty("Final Top Motor Velocity", () -> finalTopVelocity, null);
    builder.addDoubleProperty("FInal Bottom Motor Velocity", () -> finalBottomVelocity, null);
    builder.addDoubleProperty("Calculated Top kV", () -> finalTopkV, null);
    builder.addDoubleProperty("Calculated Bottom kV", () -> finalBottomkV, null);
  }

}
