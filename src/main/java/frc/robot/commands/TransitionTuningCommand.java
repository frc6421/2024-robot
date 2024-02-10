// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransitionSubsystem;

public class TransitionTuningCommand extends Command {

    private static enum Tune {
      KV,
      KS,
      KP,
      ERROR,
      DEFAULTSPEED
    }

    private final TransitionSubsystem transitionSubsystem;

    private final SendableChooser<Tune> tuneChooser;

    private double voltage;
    private double setKS;
    private double setKP;
    private double setError;
    private double setDefaultSpeed;
    private String errorString;

  /** Creates a new TransitionTuningCommand. */
  public TransitionTuningCommand(TransitionSubsystem transition) {
    transitionSubsystem = transition;

    tuneChooser = new SendableChooser<>();
    tuneChooser.setDefaultOption("KV", Tune.KV);
    tuneChooser.addOption("KS", Tune.KS);
    tuneChooser.addOption("Offset", Tune.ERROR);
    tuneChooser.addOption("Default Speed", Tune.DEFAULTSPEED);

    Shuffleboard.getTab("Transition Tuning").add(tuneChooser);
    Shuffleboard.getTab("Transition Tuning").add(this);

    // Declare subsystem dependancies
    addRequirements(transition);

    errorString = "OK";
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (tuneChooser.getSelected()) {
      case KV:
        transitionSubsystem.setTransitionVoltage(voltage);
        break;
      
      case KS:
        transitionSubsystem.setTransitionVoltage(setKS);
        break;

      case KP:
        transitionSubsystem.setTransitionMotorP(setKP);
        transitionSubsystem.setTransitionPosition(100);
        break;

      case ERROR:
        CenterNoteCommand.ERROR_MM = setError;
        break;
      
      case DEFAULTSPEED:
        CenterNoteCommand.DEFAULT_BELT_SPEED = setDefaultSpeed;
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Set KS", () -> setKS, this::setVoltage);
    builder.addDoubleProperty("Set Error", () -> setError, this::setError);
    builder.addDoubleProperty("Set Default Speed", () -> setDefaultSpeed, this::setDefaultSpeed);
    builder.addDoubleProperty("Set P", () -> setKP, this::setP);
    builder.addStringProperty("Motor Error", () -> errorString, null);
    builder.addDoubleProperty("Current Position", () -> transitionSubsystem.getPosition(), null);
  }

  private void setFeedForward() {
    
  }

  private void setVoltage(double voltage) { 
    setKS = voltage;
  }

  private void setError(double error) { 
    setError = error;
  }

  private void setDefaultSpeed(double defaultSpeed) { 
    setDefaultSpeed = defaultSpeed;
  }

  private void setP(double P) { 
    setKP = P;
  }
}
