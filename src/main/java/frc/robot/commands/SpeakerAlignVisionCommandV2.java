// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class SpeakerAlignVisionCommandV2 extends Command {
  private final DriveSubsystem driveSubsystem;

  // TODO update max velocity and acceleration
  // In m/s
  private static final double maxLinearVelocity = 2;
  private static final double maxLinearAcceleration = 2;

  // In radians/sec
  private static final double maxAngularVelocity = 2 * Math.PI;
  private static final double maxAngularAcceleration = 2 * Math.PI;

  private Pose2d targetPose;
  private Pose2d currentPose;

  // In meters
  private double allowableYError = 0.025;
  // In radians
  private double allowableAngleError = 0.04;

  private static final double yP = 2.31;
  private static final double yI = 0;
  private static final double yD = 0;

  private static final double rotationP = 5;
  private static final double rotationI = 0;
  private static final double rotationD = 0;

  private static final TrapezoidProfile.Constraints linearConstraints = new TrapezoidProfile.Constraints(
      maxLinearVelocity, maxLinearAcceleration);
  private static final TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
      maxAngularVelocity, maxAngularAcceleration);

  private static final ProfiledPIDController yController = new ProfiledPIDController(yP, yI, yD, linearConstraints);

  private static final ProfiledPIDController rotationController = new ProfiledPIDController(rotationP, rotationI,
      rotationD, rotationConstraints);

  private final SwerveRequest.FieldCentric driveRequest;

  /** Creates a new SpeakerAlignVisionCommandV2. */
  public SpeakerAlignVisionCommandV2(DriveSubsystem drive) {

    driveSubsystem = drive;

    driveRequest = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.Velocity);

    driveRequest.ForwardReference = SwerveRequest.ForwardReference.RedAlliance;

    yController.setTolerance(allowableYError);
    rotationController.setTolerance(allowableAngleError);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Optional<DriverStation.Alliance> allianceColor = DriverStation.getAlliance();

    if (allianceColor.isPresent()) {
      if (allianceColor.get().equals(Alliance.Red)) {
        targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(4).get().toPose2d();
      } else if (allianceColor.get().equals(Alliance.Blue)) {
        targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(7).get().toPose2d();
      }
    } else {
      targetPose = driveSubsystem.getCurrentPose2d();
    }

    yController.setGoal(targetPose.getY());
    rotationController.setGoal(targetPose.getRotation().getRadians());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPose = driveSubsystem.getCurrentPose2d();

    driveSubsystem.setControl(driveRequest
        .withVelocityX(0)
        .withVelocityY(yController.calculate(currentPose.getY()))
        .withRotationalRate(rotationController.calculate(currentPose.getRotation().getRadians())));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    driveSubsystem.setControl(driveRequest
      .withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(0)
    );

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return yController.atGoal() && rotationController.atGoal();
  }
}