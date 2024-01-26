// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransitionArm;

public class ArmCommand extends Command {

  TransitionArm arm;

  /** Creates a new armCommand. */
  public ArmCommand(TransitionArm armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);

    arm = armSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    switch(RobotContainer.currentArmState)
    {
      case INTAKE:
        arm.setArmMotorPosition(0); // TODO positions needed
        break;

      case SCOREING:
        arm.setArmMotorPosition(0); // TODO positions needed
        break;
    }
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
    return false;
  }
}
