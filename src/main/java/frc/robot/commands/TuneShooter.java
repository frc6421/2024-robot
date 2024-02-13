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
    topkS,
    topkV,
    topkP,
    bottomkS,
    bottomkV,
    bottomkP
  }

  private double topkS;
  private double bottomkS;
  private double topkV;
  private double bottomkV;
  private double topkP;
  private double bottomkP;
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
    tuneChooser.setDefaultOption("topkS", Tune.topkS);
    tuneChooser.addOption("bottomkS", Tune.bottomkS);
    tuneChooser.addOption("topkV", Tune.topkV);
    tuneChooser.addOption("bottomkV", Tune.bottomkV);
    tuneChooser.addOption("topkP", Tune.topkP);
    tuneChooser.addOption("bottomP", Tune.bottomkP);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    Shuffleboard.getTab("Shooter Tuning").add(this);
    Shuffleboard.getTab("Shooter Tuning").add("Tuning Chooser", tuneChooser);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    switch (tuneChooser.getSelected()) {
      case topkS:
        shooterSubsystem.setTopVoltage(topkS);
        break;
      case topkV:
        shooterSubsystem.setTopVoltage(topkS + topkV);
        break;
      case topkP:
        shooterSubsystem.setTopP(topkP);
        shooterSubsystem.setTuningMotorVelocity(setVolocity, topkS + topkV);
        break;
      //Bottom Shooter
      case bottomkS:
        shooterSubsystem.setBottomVoltage(bottomkS);
        break;
      case bottomkV:
        shooterSubsystem.setBottomVoltage(bottomkS + bottomkV);
        break;
      case bottomkP:
        shooterSubsystem.setBottomP(bottomkP);
        shooterSubsystem.setTuningMotorVelocity(setVolocity, bottomkS + bottomkV);
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
    if (tuneChooser.getSelected() == Tune.topkV || tuneChooser.getSelected() == Tune.bottomkV) {
      finalTopkV = shooterSubsystem.getTopMotorVoltage() / shooterSubsystem.getTopMotorVelocity();
      finalBottomkV = shooterSubsystem.getBottomMotorVoltage() / shooterSubsystem.getBottomMotorVelocity();
    }
    shooterSubsystem.setTopVoltage(0);
    shooterSubsystem.setBottomVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void setTopkS(double kS) {
    this.topkS = kS;
  }
  private void setBottomkS(double kS) {
    this.bottomkS = kS;
  }

  private void setTopkV(double kV) {
    this.topkV = kV;
  }

  private void setBottomkV(double kV){
    this.bottomkV = kV;
  }

  private void setTopkP(double kP) {
    this.topkP = kP;
  }

  private void setBottomkP(double kP){
    this.bottomkP = kP;
  }

  public void setVolocity(double setVolocity) {
    this.setVolocity = setVolocity;
  }

   @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("topkS", () -> topkS, this::setTopkS);
    builder.addDoubleProperty("bottomkS", () -> bottomkS, this::setBottomkS);
    builder.addDoubleProperty("topkV", () -> topkV, this::setTopkV);
    builder.addDoubleProperty("bottomkV", () -> bottomkV, this::setBottomkV);
    builder.addDoubleProperty("topkP", () -> topkP, this::setTopkP);
    builder.addDoubleProperty("bottomP", () -> bottomkP, this::setBottomkP);
    //TODO: Search how to fix this
    builder.addDoubleProperty("set Velcoty", () -> kP, this::setVolocity);
    builder.addDoubleProperty("Top Motor Velocity", () -> shooterSubsystem.getTopMotorVelocity(), null);
    builder.addDoubleProperty("Bottom Motor Velocity", () -> shooterSubsystem.getBottomMotorVelocity(), null);
    builder.addDoubleProperty("Final Top Motor Velocity", () -> finalTopVelocity, null);
    builder.addDoubleProperty("FInal Bottom Motor Velocity", () -> finalBottomVelocity, null);
    builder.addDoubleProperty("Calculated Top kV", () -> finalTopkV, null);
    builder.addDoubleProperty("Calculated Bottom kV", () -> finalBottomkV, null);
  }

}
