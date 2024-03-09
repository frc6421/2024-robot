// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Cameras;
import frc.robot.Constants.RobotStates;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class SpeakerVisionCommand extends Command {
  private DriveSubsystem driveSubsystem;

  // In meters
  private double maxAngularSpeed = 2 * Math.PI;
  private double maxAngularAcceleration = 2 * Math.PI;

  private TrapezoidProfile.Constraints angularConstraints = new Constraints(maxAngularSpeed, maxAngularAcceleration);

  // In camera degrees
  // TODO tune PID
  private ProfiledPIDController rotationController;

  private double rotationP = 1.0;

  // In camera degrees
  // TODO tune
  private double allowableRotationError = 0.5;

  private double rotationSpeed = 0.0;

  private final SwerveRequest.FieldCentric visionDriveRequest;

  private Optional<DriverStation.Alliance> allianceColor;

  private final AprilTagFieldLayout crescendoField = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private int targetTagID = 0;

  /** Creates a new SpeakerVisionCommand. */
  public SpeakerVisionCommand(DriveSubsystem drive) {
    driveSubsystem = drive;

    visionDriveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    rotationController = new ProfiledPIDController(rotationP, 0.0, 0.0, angularConstraints);

    rotationController.setTolerance(allowableRotationError);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (allianceColor.isPresent()) {

      targetTagID = allianceColor.get().equals(Alliance.Red) ? 4 : 7;

    } else {

      end(true);

    }

    if (Cameras.isTarget(Cameras.speakerCamera)) {

      // Check distance from the target using camera pitch
      for (int i = 0; i < VisionConstants.SPEAKER_PITCH_ARRAY.length; i++) {

        if (Cameras.getPitch(Cameras.speakerCamera, targetTagID) >= VisionConstants.SPEAKER_PITCH_ARRAY[i]
            && (Cameras.getPitch(Cameras.speakerCamera, targetTagID) != 180.0)) {

          // Set rotation to turn to center on the speaker
          rotationController.setGoal(VisionConstants.SPEAKER_YAW_ARRAY[i]);

          break;

        }

      }

    } else {

      end(true);

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentRotation = Cameras.getYaw(Cameras.speakerCamera, targetTagID);

    if (RobotContainer.robotState.equals(RobotStates.SPEAKER)) {

      // Ends command if no AprilTag is detected in the camera frame
      // Camera methods return 180.0 if the target tag ID is not detected
      if (currentRotation == 180.0) {

        end(true);

      } else {

        rotationSpeed = rotationController.calculate(currentRotation);

      }

    } else {

      end(true);

    }

    SmartDashboard.putNumber("Target Rotation (Yaw)", rotationController.getSetpoint().position);

    SmartDashboard.putNumber("Current Rotation (Yaw)", currentRotation);

    driveSubsystem.setControl(
        visionDriveRequest.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(rotationSpeed));

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (interrupted) {

      System.out.println("*****SpeakerVisionCommand Interrupted*****");
      System.out.println("Alliance Present: " + allianceColor.isPresent());
      System.out.println("AprilTag Pose Present: " + crescendoField.getTagPose(targetTagID).isPresent());

    }

    driveSubsystem.setControl(
        visionDriveRequest.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotationController.atGoal();
  }
}
