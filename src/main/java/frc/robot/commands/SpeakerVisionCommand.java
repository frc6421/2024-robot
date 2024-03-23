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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Cameras;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDConstants.LEDColors;

public class SpeakerVisionCommand extends Command {
  private DriveSubsystem driveSubsystem;

  // In meters
  private double maxAngularSpeed = 2 * Math.PI;

  // In camera degrees
  // TODO tune PID
  private PIDController rotationController;

  private double rotationP = 0.1;

  // In camera degrees
  // TODO tune
  private double allowableRotationError = 0.25;

  private double rotationSpeed = 0.0;

  private MedianFilter filter = new MedianFilter(10);

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

    rotationController = new PIDController(rotationP, 0.0, 0.0);

    rotationController.setTolerance(allowableRotationError);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    filter.reset();

    rotationController.reset();

    allianceColor = DriverStation.getAlliance();

    if (allianceColor.isPresent()) {

      targetTagID = allianceColor.get().equals(Alliance.Red) ? 4 : 7;

    } else {

      cancel();
      System.out.println("SpeakerVisionCommand canceled - No alliance color present");

    }

    // if (Cameras.isTarget(Cameras.speakerCamera)) {

    // // Check distance from the target using camera pitch
    // for (int i = 0; i < VisionConstants.SPEAKER_PITCH_ARRAY.length; i++) {

    // if (Cameras.getPitch(Cameras.speakerCamera, targetTagID) >=
    // VisionConstants.SPEAKER_PITCH_ARRAY[i]
    // && (Cameras.getPitch(Cameras.speakerCamera, targetTagID) != 180.0)) {

    // // Set rotation to turn to center on the speaker
    // rotationController.setSetpoint(VisionConstants.SPEAKER_YAW_ARRAY[i]);

    // break;

    // }

    // }

    // } else {

    // exitCommand = true;

    // }

    rotationController.setSetpoint(VisionConstants.SPEAKER_YAW_ANGLE);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentRotation = filter.calculate(Cameras.getYaw(Cameras.speakerCamera, targetTagID));

    // Ends command if no AprilTag is detected in the camera frame
    // Camera methods return 180.0 if the target tag ID is not detected
    if (currentRotation == 180.0) {

      cancel();
      System.out.println("SpeakerVisionCommand canceled - No AprilTag detected (speaker camera)");

    } else {

      rotationSpeed = MathUtil.clamp(rotationController.calculate(currentRotation), -maxAngularSpeed, maxAngularSpeed);

    }

    SmartDashboard.putNumber("Target Rotation (Yaw)", rotationController.getSetpoint());

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

    }

    driveSubsystem.setControl(
        visionDriveRequest.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotationController.atSetpoint();
  }
}
