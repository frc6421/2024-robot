// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class SpeakerAlignVisionCommand extends Command {

  DriveSubsystem driveSubsystem;

  //TODO update max velocity and acceleration
  private static final double maxAngularVelocity = 2 * Math.PI;
  private static final double maxAngularAcceleration = Math.PI;

  private Pose2d targetPose;
  private Pose2d currentPose;

  private double angleToTarget;
  private double currentAngle;

  private double allowableError = 2;

  //TODO update PID values
  private static final double rotationP = 0;
  private static final double rotationI = 0;
  private static final double rotationD = 0;

  private static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration);

  private static final ProfiledPIDController rotationController = new ProfiledPIDController(rotationP, rotationI, rotationD, constraints);

  private final SwerveRequest.FieldCentric driveRequest;

  /** Creates a new SpeakerAlignVisionCommand. */
  public SpeakerAlignVisionCommand(DriveSubsystem drive) {
    driveSubsystem = drive;

    driveRequest = new SwerveRequest.FieldCentric();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.setTolerance(allowableError);
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

    //TODO see what value this outputs
    //TODO determine if this needs to change sign based on alliance color
    angleToTarget = targetPose.relativeTo(currentPose).getRotation().getDegrees();

    currentAngle = driveSubsystem.getCurrentPose2d().getRotation().getDegrees();

    rotationController.setGoal(currentAngle + angleToTarget);

    driveSubsystem.setControl(
      driveRequest.withRotationalRate(rotationController.calculate(currentAngle)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setControl(
      driveRequest.withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotationController.atSetpoint();
  }
}
