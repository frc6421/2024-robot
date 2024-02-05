// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransitionSubsystem;

public class CenterNoteCommand extends Command {
  TransitionSubsystem transition;
  boolean centered;
  public static class CenterNoteConstants {
    // TODO Tune values
    private static final double DEFAULT_BELT_SPEED = 0.3;
    private static final double CENTERING_POS = 100;

  }
  /** Creates a new CenterNoteInTransition. */
  public CenterNoteCommand(TransitionSubsystem transitionSubsystem) {
    addRequirements(transitionSubsystem);

    transition = transitionSubsystem;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transition.setTransitionMotorSpeed(CenterNoteConstants.DEFAULT_BELT_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
      if (transition.timeOfFlightOut.getRange() > transition.timeOfFlightIn.getRange() + 5) {
        transition.setTransitionMotorSpeed(-1.0 * CenterNoteConstants.DEFAULT_BELT_SPEED);
        centered = false;
    }
      else if (transition.timeOfFlightOut.getRange() + 5 < transition.timeOfFlightIn.getRange()) {
        transition.setTransitionMotorSpeed(CenterNoteConstants.DEFAULT_BELT_SPEED);
        centered = false;
    }
      else {
        transition.setTransitionMotorSpeed(0);
        centered = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return centered;
  }
}
