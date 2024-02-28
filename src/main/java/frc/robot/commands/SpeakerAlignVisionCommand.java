// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class SpeakerAlignVisionCommand extends Command {

  private final DriveSubsystem driveSubsystem;

  // In rad/sec
  private static final double maxAngularVelocity = 2 * Math.PI;
  private static final double maxAngularAcceleration = 2 * Math.PI;

  private Pose2d targetPose;
  private Pose2d currentPose;

  private double calculatedAngle;
  private double currentAngle;
  private double angleToRotate;
  private double targetAngle;

  // In radians
  private double allowableError = 0.04;

  private static final double rotationP = 5;
  private static final double rotationI = 0;
  private static final double rotationD = 0;

  private static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxAngularVelocity,
      maxAngularAcceleration);

  private static final ProfiledPIDController rotationController = new ProfiledPIDController(rotationP, rotationI,
      rotationD, constraints);

  private final SwerveRequest.FieldCentric driveRequest;

  private Optional<DriverStation.Alliance> allianceColor;

  /** Creates a new SpeakerAlignVisionCommand. */
  public SpeakerAlignVisionCommand(DriveSubsystem drive) {
    driveSubsystem = drive;
    rotationController.setTolerance(allowableError);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    driveRequest = new SwerveRequest.FieldCentric();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    allianceColor = DriverStation.getAlliance();

    if (allianceColor.isPresent()) {
      if (allianceColor.get().equals(Alliance.Red)) {
        targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(4).get().toPose2d();
      } else if (allianceColor.get().equals(Alliance.Blue)) {
        targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(7).get().toPose2d();
      }
    } else {
      targetPose = driveSubsystem.getCurrentPose2d();
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPose = driveSubsystem.getCurrentPose2d();
    currentAngle = currentPose.getRotation().getRadians();

    if(allianceColor.get().equals(Alliance.Blue) && currentPose.getY() > targetPose.getY()) {
      calculatedAngle = -Math.atan((currentPose.getX() - targetPose.getX()) / (currentPose.getY() - targetPose.getY()));
    }
    


    //calculatedAngle = PhotonUtils.getYawToPose(currentPose, targetPose).getRadians();
    //calculatedAngle = targetPose.relativeTo(currentPose).getRotation().getRadians();

    targetAngle = calculatedAngle;
    //targetAngle = currentAngle - calculatedAngle;

    SmartDashboard.putNumber("currentAngle", currentPose.getRotation().getDegrees());
    SmartDashboard.putNumber("angleToTarget", Units.radiansToDegrees(calculatedAngle));

    rotationController.setGoal(targetAngle);

    SmartDashboard.putNumber("goal", Units.radiansToDegrees(rotationController.getGoal().position));

    driveSubsystem.setControl(
    driveRequest
    .withVelocityX(0)
    .withVelocityY(0)
    .withRotationalRate(rotationController.calculate(currentAngle)));
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
    return rotationController.atGoal();
  }
}
