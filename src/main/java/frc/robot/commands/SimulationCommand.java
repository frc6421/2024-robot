// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransitionArmSubsystem;
import frc.robot.subsystems.TransitionArmSubsystem.TransitionArmConstants;

public class SimulationCommand extends Command{

  TalonFXSimState motorSim;

  DCMotorSim sim;

  double goPos;

  double armVoltage;

  /** Creates a new armCommand. */
  public SimulationCommand(TransitionArmSubsystem armSubsystem, double position, double voltage) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);

    motorSim = TransitionArmSubsystem.armMotorSim;

    sim = TransitionArmSubsystem.armSim;

    goPos = position;

    armVoltage = voltage;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sim.setInputVoltage(armVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    sim.setInputVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (goPos > 0) {
    return (Units.radiansToDegrees(sim.getAngularPositionRad()) > goPos);
  } else {
    return (Units.radiansToDegrees(sim.getAngularPositionRad()) < goPos);
  }

  }
}