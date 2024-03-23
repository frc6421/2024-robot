// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Cameras;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDConstants.LEDColors;

public class AmpVisionCommand extends Command {
  private DriveSubsystem driveSubsystem;

  // In meters
  private double maxSpeed = 2.5;

  // In camera degrees
  private PIDController xController;
  private PIDController yController;

  private double xP = 0.08;
  private double xD = 0.0016;

  private double yP = 0.08;
  private double yD = 0.0016;

  private double rotationP = 5.0;

  // In camera degrees
  private double allowableXError = 0.8;
  private double allowableYError = 0.8;

  private double allowableVelocityError = 10.0;

  // In gyro radians
  private double allowableRotationError = 0.035;

  private double xSpeed = 0.0;
  private double ySpeed = 0.0;

  // Both in gyro radians
  private Rotation2d targetRotation;
  private Rotation2d averageRotation;

  private double averageRotationRadians;

  private MedianFilter xAmpFilter = new MedianFilter(5);
  private MedianFilter yAmpFilter = new MedianFilter(5);
  private MedianFilter xSpeakerFilter = new MedianFilter(5);
  private MedianFilter ySpeakerFilter = new MedianFilter(5);

  private final SwerveRequest.FieldCentricFacingAngle visionDriveRequest;

  private Optional<DriverStation.Alliance> allianceColor;

  private final AprilTagFieldLayout crescendoField = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private int targetTagID = 0;

  private boolean exitCommand = false;

  /** Creates a new AmpTrapVisionCommand. */
  public AmpVisionCommand(DriveSubsystem drive) {
    driveSubsystem = drive;

    visionDriveRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    visionDriveRequest.HeadingController.setPID(rotationP, 0.0, 0.0);
    visionDriveRequest.HeadingController.setTolerance(allowableRotationError);
    visionDriveRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    xController = new PIDController(xP, 0.0, xD);
    yController = new PIDController(yP, 0.0, yD);

    xController.setTolerance(allowableXError, allowableVelocityError);
    yController.setTolerance(allowableYError, allowableVelocityError);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    exitCommand = false;

    System.out.println("AmpVisionCommand Initialize");

    xAmpFilter.reset();
    yAmpFilter.reset();
    xSpeakerFilter.reset();
    ySpeakerFilter.reset();

    xController.reset();
    yController.reset();

    allianceColor = DriverStation.getAlliance();

    if (allianceColor.isPresent()) {

      targetTagID = (allianceColor.get().equals(Alliance.Red)) ? 5 : 6;

    } else {

      exitCommand = true;
      System.out.println("AmpVisionCommand canceled - No alliance color present");

    }

    // Set setpoint to turn the robot to the correct angle (degrees to work with the
    // drive request)
    if (crescendoField.getTagPose(targetTagID).isPresent()) {

      targetRotation = crescendoField.getTagPose(targetTagID).get().getRotation().toRotation2d();

    } else {

      exitCommand = true;
      System.out.println("AmpVisionCommand canceled - No AprilTag pose present");

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentX;
    double currentY;

    if(Cameras.isTarget(Cameras.speakerCamera) && !Cameras.isTarget(Cameras.ampCamera)) {

      currentX = xSpeakerFilter.calculate(Cameras.getYaw(Cameras.speakerCamera, targetTagID));
      currentY = ySpeakerFilter.calculate(Cameras.getPitch(Cameras.speakerCamera, targetTagID));

      // Set setpoint to center the robot on the amp in the x direction
      xController.setSetpoint(VisionConstants.AMP_FAR_YAW_ANGLE);

      // Set setpoint to drive the robot up against the amp
      yController.setSetpoint(VisionConstants.AMP_FAR_PITCH_ANGLE);

    } else {

      currentX = xAmpFilter.calculate(Cameras.getYaw(Cameras.ampCamera, targetTagID));
      currentY = yAmpFilter.calculate(Cameras.getPitch(Cameras.ampCamera, targetTagID));

      // Set setpoint to center the robot on the amp in the x direction
      xController.setSetpoint(VisionConstants.AMP_CLOSE_YAW_ANGLE);

      // Set setpoint to drive the robot up against the amp
      yController.setSetpoint(VisionConstants.AMP_CLOSE_PITCH_ANGLE);

    }

    // Ends command if no AprilTag is detected in the camera frame
    // Camera methods return 180.0 if the target tag ID is not detected
    if (currentX == 180.0 || currentY == 180.0) {

      exitCommand = true;
      System.out.println("AmpVisionCommand canceled - No AprilTag detected");

    } else {

      if (xController.atSetpoint()) {

        xSpeed = 0.0;

      } else {

        xSpeed = MathUtil.clamp(xController.calculate(currentX), -maxSpeed, maxSpeed);

      }

      if (yController.atSetpoint()) {

        ySpeed = 0.0;

      } else {

        ySpeed = MathUtil.clamp(yController.calculate(currentY), -maxSpeed, maxSpeed);

      }

    }

    averageRotationRadians = (driveSubsystem.getPose2d().getRotation().getRadians() + targetRotation.getRadians()) / 2;
    averageRotation = new Rotation2d(averageRotationRadians);

    SmartDashboard.putNumber("Target X (Yaw)", xController.getSetpoint());
    SmartDashboard.putNumber("Target Y (Pitch)", yController.getSetpoint());
    SmartDashboard.putNumber("Target Rotation (Radians)", visionDriveRequest.HeadingController.getSetpoint());

    SmartDashboard.putNumber("Current X (Yaw)", currentX);
    SmartDashboard.putNumber("Current Y (Pitch)", currentY);
    SmartDashboard.putNumber("Current Rotation (Radians)", driveSubsystem.getPigeon2().getRotation2d().getRadians());
    SmartDashboard.putNumber("Current Pose Rotation", driveSubsystem.getPose2d().getRotation().getRadians());

    SmartDashboard.putNumber("X Speed", xSpeed);
    SmartDashboard.putNumber("Y Speed", ySpeed);

    if(Math.abs(xController.getPositionError()) < 25) {

      driveSubsystem.setControl(
        visionDriveRequest.withVelocityX(-xSpeed)
            .withVelocityY(ySpeed)
            .withTargetDirection(targetRotation));
      
    } else {

      driveSubsystem.setControl(
        visionDriveRequest.withVelocityX(-xSpeed)
            .withVelocityY(ySpeed)
            .withTargetDirection(averageRotation));

    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("AmpVisionCommand End");

    if (interrupted) {

      System.out.println("*****AmpVisionCommand Interrupted*****");
      System.out.println("Alliance Present: " + allianceColor.isPresent());
      System.out.println("AprilTag Pose Present: " + crescendoField.getTagPose(targetTagID).isPresent());

    }

    driveSubsystem.setControl(
        visionDriveRequest.withVelocityX(0)
            .withVelocityY(0)
            .withTargetDirection(driveSubsystem.getPose2d().getRotation()));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xController.atSetpoint() && yController.atSetpoint() && visionDriveRequest.HeadingController.atSetpoint())
      || exitCommand;
  }
}
