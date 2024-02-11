// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class ShooterPivotTuningCommand extends Command {
  private final ShooterAngleSubsystem shooterAngleSubsystem;

  private enum Tune {
    kS  ,
    kP
  }
  private final SendableChooser<Tune> tuneChooser;

  private double kS;
  private double kP;
  private double angle;

  /** Creates a new ShooterTuningCommand. */
  public ShooterPivotTuningCommand(ShooterAngleSubsystem shooter) {
    shooterAngleSubsystem = shooter;

    tuneChooser = new SendableChooser<Tune>();
    tuneChooser.setDefaultOption("kS", Tune.kS);
    tuneChooser.addOption("kP", Tune.kP);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    Shuffleboard.getTab("Shooter Pivot Tuning").add(this);
    Shuffleboard.getTab("Shooter Pivot Tuning").add("Chooser", tuneChooser);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (tuneChooser.getSelected()) {
      case kS:
        shooterAngleSubsystem.setMotorVoltage(kS);
        break;
      case kP:
        shooterAngleSubsystem.setP(kP);
        shooterAngleSubsystem.setAngle(angle, kS);
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterAngleSubsystem.setMotorVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void setkS(double kS) {
    this.kS = kS;
  }

  private void setkP(double kP) {
    this.kP = kP;
  }

  private void setAngle(double angle) {
    this.angle = angle;
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("kS", () -> kS, this::setkS);
    builder.addDoubleProperty("kP", () -> kP, this::setkP);
    builder.addDoubleProperty("Angle", () -> angle, this::setAngle);
    builder.addDoubleProperty("Current Angle", () -> shooterAngleSubsystem.getAngleEncoderPostition(), null);
  }

}
