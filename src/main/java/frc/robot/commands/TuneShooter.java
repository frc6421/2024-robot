// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class TuneShooter extends Command {

  private enum Tune {
    kS,
    kV,
    kP
  }

  private double kS;
  private double kV;
  private double kP;
  private double setVolocity;
  private double finalTopVelocity;
  private double finalBottomVelocity;
  private double finalTopkV;
  private double finalBottomkV;

  private final SendableChooser<Tune> tuneChooser;

  private final ShooterSubsystem shooterSubsystem;
  /** Creates a new TuneShooter. */
  public TuneShooter(ShooterSubsystem shooter) {
    shooterSubsystem = shooter;

    tuneChooser = new SendableChooser<Tune>();
    tuneChooser.setDefaultOption("kS", Tune.kS);
    tuneChooser.addOption("kV", Tune.kV);
    tuneChooser.addOption("kP", Tune.kP);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    Shuffleboard.getTab("Shooter Tuning").add(this);
    Shuffleboard.getTab("Shooter Tuning").add("Tuning Chooser", tuneChooser);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    switch (tuneChooser.getSelected()) {
      case kS:
        shooterSubsystem.setVoltage(kS);
        break;
      case kV:
        shooterSubsystem.setVoltage(kS + kV);
        break;
      case kP:
        shooterSubsystem.setP(kP);
        shooterSubsystem.setTuningMotorVelocity(setVolocity, kS + kV);
        break;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    finalTopVelocity = shooterSubsystem.getTopMotorVelocity();
    finalBottomVelocity = shooterSubsystem.getBottomMotorVelocity();
    if (tuneChooser.getSelected() == Tune.kV) {
      finalTopkV = shooterSubsystem.getTopMotorVoltage() / shooterSubsystem.getTopMotorVelocity();
      finalBottomkV = shooterSubsystem.getBottomMotorVoltage() / shooterSubsystem.getBottomMotorVelocity();
    }
    shooterSubsystem.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void setkS(double kS) {
    this.kS = kS;
  }

  private void setkV(double kV) {
    this.kV = kV;
  }

  private void setkP(double kP) {
    this.kP = kP;
  }

  public void setVolocity(double setVolocity) {
    this.setVolocity = setVolocity;
  }

   @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("kS", () -> kS, this::setkS);
    builder.addDoubleProperty("kV", () -> kV, this::setkV);
    builder.addDoubleProperty("kP", () -> kP, this::setkP );
    builder.addDoubleProperty("set Velcoty", () -> kP, this::setVolocity);
    builder.addDoubleProperty("Top Motor Velocity", () -> shooterSubsystem.getTopMotorVelocity(), null);
    builder.addDoubleProperty("Bottom Motor Velocity", () -> shooterSubsystem.getBottomMotorVelocity(), null);
    builder.addDoubleProperty("Final Top Motor Velocity", () -> finalTopVelocity, null);
    builder.addDoubleProperty("FInal Bottom Motor Velocity", () -> finalBottomVelocity, null);
    builder.addDoubleProperty("Calculated Top kV", () -> finalTopkV, null);
    builder.addDoubleProperty("Calculated Bottom kV", () -> finalBottomkV, null);
  }

}
