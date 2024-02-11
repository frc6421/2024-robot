// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class ShooterAngleCommand extends Command {
  /** Creates a new ShooterAngleCommand. */

  Timer timer = new Timer();

  ShooterAngleSubsystem angle;

  private TrapezoidProfile.State angleGoal = new TrapezoidProfile.State();
  
  private TrapezoidProfile.State angleSetpoint = new TrapezoidProfile.State();

  TrapezoidProfile angleProfile;

  public ShooterAngleCommand(ShooterAngleSubsystem angleSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(angleSubsystem);
    
    angle = angleSubsystem;
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
    angleSetpoint = angleProfile.calculate(timer.get(), new TrapezoidProfile.State(angle.getAngleEncoderPostition(), 0), angleGoal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angle.setAngle(angleSetpoint.position);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() > angleProfile.totalTime());
  }
}
