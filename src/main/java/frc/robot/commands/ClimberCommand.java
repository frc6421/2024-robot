// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ClimberSubsystem;


public class ClimberCommand extends Command {

  Timer timer = new Timer();

  ClimberSubsystem climber;

  private final TrapezoidProfile.Constraints climberConstraints = 
    new TrapezoidProfile.Constraints(600, 250);

  private TrapezoidProfile.State climberGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State climberSetPoint = new TrapezoidProfile.State();

  TrapezoidProfile climberProfile;

  /** Creates a new ClimberCommand. */
  public ClimberCommand(ClimberSubsystem climberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubsystem);

    climber = climberSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberSetPoint = climberProfile.calculate(timer.get(), new TrapezoidProfile.State(climber.getClimberMotorPosition(), 0), climberGoal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setClimberMotorPosition(climberSetPoint.position);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > climberProfile.totalTime();
  }
}
