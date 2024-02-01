// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransitionSubsystem;

public class CenterNoteInTransition extends Command {
  TransitionSubsystem transition;
  /** Creates a new CenterNoteInTransition. */
  public CenterNoteInTransition(TransitionSubsystem transitionSubsystem) {
    addRequirements(transitionSubsystem);

    transition = transitionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ()
    return false;
  }
}
