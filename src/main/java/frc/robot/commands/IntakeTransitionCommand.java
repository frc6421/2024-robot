// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotStates;
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

    RobotContainer.state = RobotStates.INTAKE;

    possibleOverShoot = false;

    counter = 0;

    transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED);
    intakeSubsystem.setIntakeVoltage(IntakeConstants.INTAKE_IN_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(transitionSubsystem.timeOfFlightIn.getRange() <= TransitionConstants.DETECTION_DISTANCE_MM && 
       transitionSubsystem.timeOfFlightOut.getRange() >= TransitionConstants.DETECTION_DISTANCE_MM)
    {
      transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED / 2);
    }
    if(transitionSubsystem.timeOfFlightIn.getRange() <= TransitionConstants.DETECTION_DISTANCE_MM && 
       transitionSubsystem.timeOfFlightOut.getRange() <= TransitionConstants.DETECTION_DISTANCE_MM)
    {
      intakeSubsystem.setIntakeVoltage(0);
      transitionSubsystem.setTransitionVoltage(0);
      counter++;
    }
    if(transitionSubsystem.timeOfFlightIn.getRange() >= TransitionConstants.DETECTION_DISTANCE_MM && 
       transitionSubsystem.timeOfFlightOut.getRange() <= TransitionConstants.DETECTION_DISTANCE_MM)
    {
      // TODO LED pink
      transitionSubsystem.setTransitionVoltage((-1.0 * TransitionConstants.TRANSITION_SPEED) / 4);
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

    RobotContainer.state = RobotStates.DRIVE;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (counter >= 30) 
    || ((transitionSubsystem.timeOfFlightIn.getRange() > TransitionConstants.DETECTION_DISTANCE_MM && 
         transitionSubsystem.timeOfFlightOut.getRange() > TransitionConstants.DETECTION_DISTANCE_MM) && possibleOverShoot);
  }
}
