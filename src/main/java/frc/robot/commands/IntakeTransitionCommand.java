// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;
import frc.robot.subsystems.TransitionSubsystem.TransitionConstants;

public class IntakeTransitionCommand extends Command {

  private final TransitionSubsystem transitionSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private boolean possibleOverShoot;

  private int counter;

  /** Creates a new CenterNoteInTransition. */
  public IntakeTransitionCommand(TransitionSubsystem transitionSubsystem, IntakeSubsystem intakeSubsystem) {
    
    addRequirements(transitionSubsystem, intakeSubsystem);

    this.transitionSubsystem = transitionSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    possibleOverShoot = false;

    counter = 0;

    transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED);
    intakeSubsystem.setIntakeVoltage(IntakeConstants.INTAKE_IN_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(transitionSubsystem.timeOfFlightIn.getRange() <= 350 && transitionSubsystem.timeOfFlightOut.getRange() >= 350)
    {
      transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED / 2);
      //TODO Can we turn off intake here?
      //intakeSubsystem.setIntakeVoltage(0);
    }
    if(transitionSubsystem.timeOfFlightIn.getRange() <= 350 && transitionSubsystem.timeOfFlightOut.getRange() <= 350)
    {
      intakeSubsystem.setIntakeVoltage(0);
      transitionSubsystem.setTransitionVoltage(0);
      counter++;
    }
    if(transitionSubsystem.timeOfFlightIn.getRange() >= 350 && transitionSubsystem.timeOfFlightOut.getRange() <= 350)
    {
      // TODO LED pink
      transitionSubsystem.setTransitionVoltage((-1.0 * TransitionConstants.TRANSITION_SPEED) / 4);
      System.out.println("Counter: " + counter);
      counter = 0;
      possibleOverShoot = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    // TODO LED pink
    intakeSubsystem.setIntakeVoltage(0);
    transitionSubsystem.setTransitionVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (counter >= 30) 
    || ((transitionSubsystem.timeOfFlightIn.getRange() > 350 && transitionSubsystem.timeOfFlightOut.getRange() > 350) && possibleOverShoot);
  }
}
