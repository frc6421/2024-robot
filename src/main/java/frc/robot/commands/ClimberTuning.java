// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberTuning extends Command {
  public enum ClimberTuningState{
    KP,
    KS,
    KG
  }

  private SendableChooser<ClimberTuningState> chooser;

  private final ClimberSubsystem climberSubsystem;

  private double KP;
  private double KS;
  private double KG;
  private double setPosition;

  /** Creates a new ClimberTuning. */
  public ClimberTuning(ClimberSubsystem climberSubsystem) {

    chooser = new SendableChooser<ClimberTuningState>();
    chooser.setDefaultOption("KP", ClimberTuningState.KP);
    chooser.addOption("KS", ClimberTuningState.KS);
    chooser.addOption("KG", ClimberTuningState.KG);

    this.climberSubsystem = climberSubsystem;

    Shuffleboard.getTab("Climber Tuning").add(this);
    Shuffleboard.getTab("Climber Tuning").add(chooser);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    switch(chooser.getSelected())
    {
      case KP:
        climberSubsystem.setClimberP(KP);
        climberSubsystem.setClimberMotorPosition(setPosition);
        break;
      case KS:
        climberSubsystem.setClimberVoltage(KS);
        break;
      case KG:
        climberSubsystem.setClimberVoltage(KG);
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("Climber Position", () -> climberSubsystem.getClimberMotorPosition(), null);
    builder.addDoubleProperty("Set Position", () -> setPosition, this::setPositionValue);
    builder.addDoubleProperty("Climber P Setting", () -> KP, this::setP);
    builder.addDoubleProperty("Climber KS Setting", () -> KS, this::setKS);
    builder.addDoubleProperty("Climber KG Setting", () -> KG, this::setKG);
    builder.addDoubleProperty("Climber Motor Left", () -> climberSubsystem.getClimberLeftMotorPosition(), null);
    builder.addDoubleProperty("Climber Motor Right", () -> climberSubsystem.getClimberRightMotorPosition(), null);
  }

  public void setP(double P)
  {
    KP = P;
  }
  public void setKS(double S)
  {
    KS = S;
  }
  public void setKG(double G)
  {
    KG = G;
  }
  public void setPositionValue(double value)
  {
    setPosition = value;
  }
}
