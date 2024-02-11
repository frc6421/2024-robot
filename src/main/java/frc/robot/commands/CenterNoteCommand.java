// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransitionSubsystem;

public class CenterNoteCommand extends Command {
  private final TransitionSubsystem transition;
  boolean centered;
  public static class CenterNoteConstants {
    public static final double DEFAULT_BELT_SPEED = 0.6;
  }

  /** Creates a new CenterNoteInTransition. */
  public CenterNoteCommand(TransitionSubsystem transitionSubsystem) {
    addRequirements(transitionSubsystem);
    transition = transitionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transition.setTransitionMotorOutput(CenterNoteConstants.DEFAULT_BELT_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (transition.timeOfFlightIn.getRange() < 350 && transition.timeOfFlightOut.getRange() > 370) {
      transition.setTransitionMotorOutput(CenterNoteConstants.DEFAULT_BELT_SPEED / 4.0);
    }
    else if (transition.timeOfFlightIn.getRange() > 370 && transition.timeOfFlightOut.getRange() < 350) {
      transition.setTransitionMotorOutput(-1.0 * CenterNoteConstants.DEFAULT_BELT_SPEED / 4.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transition.setTransitionMotorOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (transition.timeOfFlightIn.getRange() < 350 && transition.timeOfFlightOut.getRange() < 350);
  }
}
