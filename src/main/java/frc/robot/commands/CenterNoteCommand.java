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
    
    // Error moved to public. default speed moved to public
  }

  public static double ERROR_MM = 5;
  public static double DEFAULT_BELT_SPEED = 0.3;
  /** Creates a new CenterNoteInTransition. */
  public CenterNoteCommand(TransitionSubsystem transitionSubsystem) {
    addRequirements(transitionSubsystem);

    transition = transitionSubsystem;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transition.setTransitionMotorSpeed(DEFAULT_BELT_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
      if (transition.timeOfFlightOut.getRange() > transition.timeOfFlightIn.getRange() + ERROR_MM) {
        transition.setTransitionMotorSpeed(-1.0 * DEFAULT_BELT_SPEED);
        centered = false;
    }
      else if (transition.timeOfFlightOut.getRange() + ERROR_MM < transition.timeOfFlightIn.getRange()) {
        transition.setTransitionMotorSpeed(DEFAULT_BELT_SPEED);
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
