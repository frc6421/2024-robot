// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterTuningCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;

  private double kS;
  private double kV;
  private double finalkVTop;
  private double finalkVBottom;
  private double kP;
  private double velocity;

  private final SendableChooser<Tune> chooser;

  private enum Tune {
    kS,
    kV,
    kP
  }

  /** Creates a new ShooterTuningCommand. */
  public ShooterTuningCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);

    chooser = new SendableChooser<Tune>();
    chooser.setDefaultOption("kS", Tune.kS);
    chooser.addOption("kV", Tune.kV);
    chooser.addOption("kP", Tune.kP);

    Shuffleboard.getTab("Shooter Tuning").add("Shooter Command", this);
    Shuffleboard.getTab("Shooter Tuning").add("Chooser", chooser);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (chooser.getSelected()) {
      case kS:
        shooterSubsystem.setTopVoltage(kS);
        shooterSubsystem.setBottomVoltage(kS);
        break;
      case kV:
        shooterSubsystem.setTopVoltage(kV);
        shooterSubsystem.setBottomVoltage(kV);
        break;
      case kP:
        shooterSubsystem.setTopConfig(kP);
        shooterSubsystem.setBottomConfig(kP);
        shooterSubsystem.setShooterMotorVelocity(velocity);
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (chooser.getSelected() == Tune.kV) {
      finalkVTop = shooterSubsystem.getTopMotorVoltage() / shooterSubsystem.getTopMotorVelocity();
      finalkVBottom = shooterSubsystem.getBottomMotorVoltage() / shooterSubsystem.getBottomMotorVelocity();
    }
    shooterSubsystem.setTopVoltage(0);
    shooterSubsystem.setBottomVoltage(0);
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

  private void setVelocity(double velocity) {
    this.velocity = velocity;
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("1. Set kS", () -> kS, this::setkS);
    builder.addDoubleProperty("2. Set kV", () -> kV, this::setkV);
    builder.addDoubleProperty("3. Set kP", () -> kP, this::setkP);
    builder.addDoubleProperty("4. Set Velocity", () -> velocity, this::setVelocity);
    builder.addDoubleProperty("Current Top Velocity", () ->shooterSubsystem.getTopMotorVelocity(), null);
    builder.addDoubleProperty("Current Bottom Velocity", () ->shooterSubsystem.getBottomMotorVelocity(), null);
    builder.addDoubleProperty("Current Top Voltage", () ->shooterSubsystem.getTopMotorVoltage(), null);
    builder.addDoubleProperty("Current Bottom Voltage", () ->shooterSubsystem.getBottomMotorVoltage(), null);
    builder.addDoubleProperty("Final Top Volts/Velocity", () -> finalkVTop, null);
    builder.addDoubleProperty("Final Bottom Volts/Velocity", () -> finalkVBottom, null);
  }

}
