// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransitionArmSubsystem;

public class ArmCommand extends Command{

  TransitionArmSubsystem arm;

  Timer timer = new Timer();

  private final TrapezoidProfile.Constraints armConstraints = 
    new TrapezoidProfile.Constraints(2400, 800);

  private TrapezoidProfile.State armGoal = new TrapezoidProfile.State();
  
  private TrapezoidProfile.State armSetpoint = new TrapezoidProfile.State();

  TrapezoidProfile armProfile;

  private double goPos;

  private int slot;

  /** Creates a new armCommand. */
  public ArmCommand(TransitionArmSubsystem armSubsystem, double position, int slot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);

    this.slot = slot;

    arm = armSubsystem;

    goPos = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    timer.reset();

    armGoal = new TrapezoidProfile.State(goPos, 0);

    armProfile = new TrapezoidProfile(armConstraints);

    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSetpoint = armProfile.calculate(timer.get(), new TrapezoidProfile.State(arm.getArmMotorPositionDeg(), 0), armGoal);
    arm.setArmMotorPosition(armSetpoint.position, slot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    arm.setArmMotorPosition(armSetpoint.position, slot);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() > armProfile.totalTime());
  }
}