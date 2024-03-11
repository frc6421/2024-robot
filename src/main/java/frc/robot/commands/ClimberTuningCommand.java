// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberTuningCommand extends Command {

  /** Creates a new ClimberTuningCommand. */
  ClimberSubsystem climberSubsystem;
  double kP;
  double voltage;
  double pos;
  
  private enum  ClimberTuningEnum {
    KP,
    VOLTAGE
  }

  private SendableChooser<ClimberTuningEnum> chooser; 

  public ClimberTuningCommand(ClimberSubsystem climberSubsystem) {
    addRequirements(climberSubsystem);

    this.climberSubsystem = climberSubsystem;

    chooser = new SendableChooser<ClimberTuningEnum>();

    chooser.setDefaultOption("KP", ClimberTuningEnum.KP);
    chooser.addOption("VOLTAGE", ClimberTuningEnum.VOLTAGE);

    Shuffleboard.getTab("Climber Tuning").add(this);
    Shuffleboard.getTab("Climber Tuning").add(chooser);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(chooser.getSelected()) {
    
      case KP:
        climberSubsystem.setP(kP);
        climberSubsystem.setClimberMotorPosition(pos);
        break;
        
      case VOLTAGE:
        climberSubsystem.setVoltage(voltage);
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    climberSubsystem.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("kP", () -> kP, this::setKP);
    builder.addDoubleProperty("voltage", () -> voltage, this::setVoltage);
    builder.addDoubleProperty("Set Position", () -> pos, this::setPos);
    builder.addDoubleProperty("Current Position", () -> climberSubsystem.getClimberMotorPosition(), null);
  }

  public void setKP(double p)
  {
    kP = p;
  }
  public void setVoltage(double voltage)
  {
    this.voltage = voltage;
  }
  public void setPos(double pos)
  {
    this.pos = pos;
  }
}