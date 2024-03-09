// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TransitionArmSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.subsystems.TransitionSubsystem.TransitionConstants;

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
    TRAP // Raise transition arm to trap degree, spit out note
  }
  private final static double TRANSITION_ARM_MID_ANGLE = 25.0;
  private final static double TRANSITION_ARM_HIGH_ANGLE = 90.0;
  private final static double TRANSITION_ARM_TRAP_ANGLE = 110.0;
  private final static double CLIMBER_LOW_ROTATIONS = 0.0;
  private final static double CLIMBER_MID_ROTATIONS = 0.0;
  private final static double CLIMBER_HIGH_ROTATIONS = 0.0;

  private ClimberStates climberStates;
  public ClimberDanceCommand(ClimberSubsystem climberSubsystem, DriveSubsystem driveSubsystem, TransitionArmSubsystem transitionArmSubsystem, TransitionSubsystem transitionSubsystem) {
    addRequirements(climberSubsystem, driveSubsystem, transitionArmSubsystem, transitionSubsystem);

    this.climberSubsystem = climberSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.transitionArmSubsystem = transitionArmSubsystem;
    this.transitionSubsystem = transitionSubsystem;
    climberStates = ClimberStates.PREPARE_CLIMB;
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
        transitionArmSubsystem.setArmMotorPosition(TRANSITION_ARM_MID_ANGLE);
        climberSubsystem.setClimberMotorPosition(CLIMBER_MID_ROTATIONS);
        // TODO DRIVE
        // Wheels to coast, Point wheels toward the stage, Drive back 
        climberStates = ClimberStates.ARMS_HIGH;
      break;
      case ARMS_HIGH:
        transitionArmSubsystem.setArmMotorPosition(TRANSITION_ARM_HIGH_ANGLE);
        climberSubsystem.setClimberMotorPosition(CLIMBER_HIGH_ROTATIONS);
        climberStates = ClimberStates.CLIMB;
      break;
      case CLIMB:
        climberSubsystem.setClimberMotorPosition(CLIMBER_LOW_ROTATIONS);
        climberStates = ClimberStates.TRAP;
      break;
      case TRAP:
        transitionArmSubsystem.setArmMotorPosition(TRANSITION_ARM_TRAP_ANGLE);
        transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED);
        new WaitCommand(0.4);
        transitionSubsystem.stopTransition();
        // TODO Reverse climb?
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
