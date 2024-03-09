// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TransitionArmSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

public class ClimberDanceCommand extends Command {
  /** Creates a new ClimberDance. */
  private final ClimberSubsystem climberSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final TransitionArmSubsystem transitionArmSubsystem;
  private final TransitionSubsystem transitionSubsystem;
  private static enum ClimberStates {
    NOT_CLIMBING, // State when the robot is not climbing/trapping
    PREPARE_CLIMB, // Raise both arms up to middle position, drive backwards.
    ARMS_HIGH, // Raise both arms to the climbing position
    CLIMB, // Lower climber arms
    TRAP, // Raise transition arm to tra pdegree, spit out note
  }
  private ClimberStates climberStates;
  public ClimberDanceCommand(ClimberSubsystem climberSubsystem, DriveSubsystem driveSubsystem, TransitionArmSubsystem transitionArmSubsystem, TransitionSubsystem transitionSubsystem) {
    addRequirements(climberSubsystem, driveSubsystem, transitionArmSubsystem, transitionSubsystem);

    this.climberSubsystem = climberSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.transitionArmSubsystem = transitionArmSubsystem;
    this.transitionSubsystem = transitionSubsystem;
    climberStates = ClimberStates.NOT_CLIMBING;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(climberStates) {
      case PREPARE_CLIMB:

      break;
      case ARMS_HIGH:

      break;
      case CLIMB:

      break;
      case TRAP:

      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
