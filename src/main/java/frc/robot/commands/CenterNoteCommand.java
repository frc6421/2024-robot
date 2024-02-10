// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.subsystems.TransitionSubsystem.TransitionConstants;

public class CenterNoteCommand extends Command {
  private final TransitionSubsystem transition;
  boolean centered;
  public static class CenterNoteConstants {
    // TODO Tune values
    
    // Error moved to public. default speed moved to public
  }

  public static double ERROR_MM = 5;
  public static double DEFAULT_BELT_SPEED = 0.85;
  /** Creates a new CenterNoteInTransition. */
  public CenterNoteCommand(TransitionSubsystem transitionSubsystem) {
    addRequirements(transitionSubsystem);

    transition = transitionSubsystem;
    
    Shuffleboard.getTab("Center Note Tuning").add(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transition.setTransitionMotorOutput(DEFAULT_BELT_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if (transition.isNoteDetected()) {
    //   transition.setTransitionMotorOutput(DEFAULT_BELT_SPEED / 2);
    //   if (transition.timeOfFlightOut.getRange() <= TransitionConstants.DETECTION_DISTANCE_MM) {

    //   }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transition.setTransitionMotorOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return transition.timeOfFlightOut.getRange() >= 100 - ERROR_MM && transition.timeOfFlightOut.getRange() <= 100 - ERROR_MM;
  }
}
