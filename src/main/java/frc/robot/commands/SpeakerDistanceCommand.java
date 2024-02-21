// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class SpeakerDistanceCommand extends Command {
  DriveSubsystem driveSubsystem;

  public double distanceToSpeaker;

  private Pose2d targetPose;
  private Pose2d currentPose;

  /** Creates a new SpeakerDistanceCommand. */
  public SpeakerDistanceCommand(DriveSubsystem drive) {
    driveSubsystem = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<DriverStation.Alliance> allianceColor = DriverStation.getAlliance();

    if(allianceColor.get().equals(Alliance.Red)) {
      targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(4).get().toPose2d();
    } else if(allianceColor.get().equals(Alliance.Blue)) {
      targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(7).get().toPose2d();
    }

    currentPose = driveSubsystem.getCurrentPose2d();

    distanceToSpeaker = Math.sqrt(Math.pow((currentPose.getX() - targetPose.getX()), 2) 
      + Math.pow((currentPose.getY() - targetPose.getY()), 2));

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
