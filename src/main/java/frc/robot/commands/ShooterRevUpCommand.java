// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterConstants;

public class ShooterRevUpCommand extends Command {

  private final ShooterSubsystem shooterSubsystem;

  private Timer time;

  private double top;
  private double bottom;

  /** Creates a new ShooterRevUp. */
  public ShooterRevUpCommand(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(shooterSubsystem);

    time = new Timer();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //shooterSubsystem.setShooterMotorVelocity(ShooterConstants.SHOOTER_SUB_RPM);
    shooterSubsystem.setTopShooterMotorVelocity(2000);
    shooterSubsystem.setBottomShooterMotorVelocity(2000);

    time.reset();
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    time.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO determine if we need a timer condition
    return ((shooterSubsystem.getBottomMotorVelocity() >= ShooterConstants.SHOOTER_SUB_RPM - 50 && 
    shooterSubsystem.getTopMotorVelocity() >= ShooterConstants.SHOOTER_SUB_RPM - 50));
  }
  
  public void setTop(double top) {
      this.top = top;
  }

  public void setBottom(double bottom) {
      this.bottom = bottom;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
      super.initSendable(builder);
      builder.addDoubleProperty("Top", null, this::setTop);
      builder.addDoubleProperty("Bottom",null, this::setBottom);

  }
}