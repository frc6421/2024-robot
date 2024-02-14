// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class TuneShooter extends Command {

  private double setVelocity;
  private double finalTopVelocity;
  private double finalBottomVelocity;
  private double finalTopkV;
  private double finalBottomkV;
  
  public SimpleWidget topkPWidget;
  public SimpleWidget topkSWidget;
  public SimpleWidget topkVWidget;

  public SimpleWidget readTopkPWidget;

  private double topkP;
  private double topkS;
  private double topkV;

  private final ShooterSubsystem shooterSubsystem;

  /** Creates a new TuneShooter. */
  public TuneShooter(ShooterSubsystem shooter) {

    shooterSubsystem = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    final ShuffleboardTab tab = Shuffleboard.getTab("Shooter Tuning");
    tab.add(this);
    topkPWidget = tab.add("Top kP", shooterSubsystem.topShooterConfig.Slot0.kP).withWidget(BuiltInWidgets.kTextView);
    topkSWidget = tab.add("Top kS", shooterSubsystem.topShooterConfig.Slot0.kS).withWidget(BuiltInWidgets.kTextView);
    topkVWidget = tab.add("Top kV", shooterSubsystem.topShooterConfig.Slot0.kV).withWidget(BuiltInWidgets.kTextView);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    topkP = topkPWidget.getEntry().getDouble(shooterSubsystem.topShooterConfig.Slot0.kP);
    topkS = topkSWidget.getEntry().getDouble(shooterSubsystem.topShooterConfig.Slot0.kS);
    topkV = topkVWidget.getEntry().getDouble(shooterSubsystem.topShooterConfig.Slot0.kV);
    shooterSubsystem.setTopConfig(topkP, topkS, topkV);
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
    builder.addDoubleProperty("Read Top kP", () -> shooterSubsystem.topShooterConfig.Slot0.kP, null);
    builder.addDoubleProperty("Read Top kS", () -> shooterSubsystem.topShooterConfig.Slot0.kS, null);
    builder.addDoubleProperty("Read Top kV", () -> shooterSubsystem.topShooterConfig.Slot0.kV, null);
    builder.addDoubleProperty("Read Bottom kP", () -> shooterSubsystem.bottomShooterConfig.Slot0.kP, null);
    builder.addDoubleProperty("Read Bottom kS", () -> shooterSubsystem.bottomShooterConfig.Slot0.kS, null);
    builder.addDoubleProperty("Read Bottom kV", () -> shooterSubsystem.bottomShooterConfig.Slot0.kV, null);
    builder.addDoubleProperty("Set Velocity", () -> setVelocity, null);
    builder.addDoubleProperty("Top Motor Velocity", () -> shooterSubsystem.getTopMotorVelocity(), null);
    builder.addDoubleProperty("Bottom Motor Velocity", () -> shooterSubsystem.getBottomMotorVelocity(), null);
    builder.addDoubleProperty("Final Top Motor Velocity", () -> finalTopVelocity, null);
    builder.addDoubleProperty("FInal Bottom Motor Velocity", () -> finalBottomVelocity, null);
    builder.addDoubleProperty("Calculated Top kV", () -> finalTopkV, null);
    builder.addDoubleProperty("Calculated Bottom kV", () -> finalBottomkV, null);
  }

}
