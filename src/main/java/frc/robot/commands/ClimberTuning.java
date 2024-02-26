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

  ClimberSubsystem climberSubsystem;

  private double setVoltage;
  private double kP;
  private double setPos;

  private SendableChooser<ClimberTune> chooser;

  public enum ClimberTune
  {
    KP,
    VOLTAGE
  }

  /** Creates a new ClimberTuning. */
  public ClimberTuning(ClimberSubsystem climberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberSubsystem = climberSubsystem;

    chooser = new SendableChooser<ClimberTune>();

    chooser.setDefaultOption("KP", ClimberTune.KP);
    chooser.addOption("Voltage", ClimberTune.VOLTAGE);

    addRequirements(climberSubsystem);

    //Shuffleboard.getTab("Climber Tuning").add(this);
    //Shuffleboard.getTab("Climber Tuning").add(chooser);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    switch(chooser.getSelected())
    {
      case KP:
        climberSubsystem.setClimbP(kP);
        climberSubsystem.setClimberMotorPosition(setPos);
        break;

      case VOLTAGE:
        climberSubsystem.setClimbVoltage(setVoltage);
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    climberSubsystem.setClimberMotorPosition(setPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    climberSubsystem.setClimbVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void initSendable(SendableBuilder builder)
  {
    super.initSendable(builder);
    builder.addDoubleProperty("Voltage Setting:", () -> setVoltage, this::setVoltage);
    builder.addDoubleProperty("Position Setting:", () -> setPos, this::setPosition);
    builder.addDoubleProperty("kP Setting:", () -> kP, this::setP);
  }

  public void setPosition(double pos)
  {
    setPos = pos;
  }

  public void setVoltage(double volts)
  {
    setVoltage = volts;
  }

  public void setP(double p)
  {
    kP = p;
  }
}
