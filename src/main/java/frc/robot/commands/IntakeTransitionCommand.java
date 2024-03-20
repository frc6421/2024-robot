// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotStates;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;
import frc.robot.subsystems.LEDSubsystem.LEDConstants.LEDColors;
import frc.robot.subsystems.TransitionSubsystem.TransitionConstants;

public class IntakeTransitionCommand extends Command {

  private final TransitionSubsystem transitionSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private boolean hasTOFOut;

  //private Timer timer;

  /** Creates a new CenterNoteInTransition. */
  public IntakeTransitionCommand(TransitionSubsystem transitionSubsystem, IntakeSubsystem intakeSubsystem) {
    System.out.println("IntakeTransition Constructor");

    this.transitionSubsystem = transitionSubsystem;
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(transitionSubsystem, intakeSubsystem);


    //timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("IntakeTransition Init");
    LEDSubsystem.setColor(LEDColors.YELLOW);
    // timer.reset();
    // timer.start();

    RobotContainer.robotState = RobotStates.INTAKE;

    hasTOFOut = false;

    System.out.println("Init ToF In: " + transitionSubsystem.getTOFInRange());
    System.out.println("init ToF Out: " + transitionSubsystem.getTOFOutRange());

    if(transitionSubsystem.getTOFInRange() >= TransitionConstants.INIT_DETECTION_DISTANCE_MM || 
       transitionSubsystem.getTOFOutRange() >= TransitionConstants.INIT_DETECTION_DISTANCE_MM)
    {
      transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED);
      intakeSubsystem.setIntakeVoltage(IntakeConstants.INTAKE_IN_SPEED);
    }
    else
    {
      System.out.println("*** ToF stopping in init!!!! ***");

      transitionSubsystem.setTransitionVoltage(0);
      intakeSubsystem.setIntakeVoltage(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(transitionSubsystem.getTOFInRange() <= TransitionConstants.DETECTION_DISTANCE_MM && !hasTOFOut)
    {
      LEDSubsystem.setColor(LEDColors.GREEN);
      transitionSubsystem.setTransitionVoltage(3);
    }

    if(transitionSubsystem.getTOFOutRange() <= TransitionConstants.DETECTION_DISTANCE_MM)
    {
      LEDSubsystem.setColor(LEDColors.HOT_PINK);
      intakeSubsystem.setIntakeVoltage(0);
      hasTOFOut = true;
      transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_CENTER_SPEED);
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    LEDSubsystem.setColor(LEDColors.HOT_PINK);
    intakeSubsystem.setIntakeVoltage(0);
    transitionSubsystem.setTransitionVoltage(0);

    RobotContainer.robotState = RobotStates.DRIVE;

    System.out.println("IntakeTransition End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Finish ToF In: " + transitionSubsystem.getTOFInRange());
    System.out.println("Finish ToF Out: " + transitionSubsystem.getTOFOutRange());

    return (transitionSubsystem.getTOFOutRange() >= TransitionConstants.DETECTION_DISTANCE_MM && hasTOFOut); 
    //|| (timer.get() >= 0.5);
  }
}
