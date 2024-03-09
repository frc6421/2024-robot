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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Cameras;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotStates;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveConstants;

public class AmpVisionCommand extends Command {
  private DriveSubsystem driveSubsystem;

  // In meters
  private double maxSpeed = 2;

  // In camera degrees
  // TODO tune PID
  private PIDController xController;
  private PIDController yController;

  private double xP = 0.1;
  private double yP = 0.1;
  private double rotationP = 5.0;

  // TODO tune these error ranges
  // In camera degrees
  private double allowableXError = 0.25;
  private double allowableYError = 0.25;

  // In gyro radians
  private double allowableRotationError = 0.035;

  private double xSpeed = 0.0;
  private double ySpeed = 0.0;

  private Rotation2d targetRotation;

  private MedianFilter xFilter = new MedianFilter(10);
  private MedianFilter yFilter = new MedianFilter(10);

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

    xController = new PIDController(xP, 0.0, 0.0);
    yController = new PIDController(yP, 0.0, 0.0);

    xController.setTolerance(allowableXError);
    yController.setTolerance(allowableYError);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    exitCommand = false;

    xFilter.reset();
    yFilter.reset();

    xController.reset();
    yController.reset();

    allianceColor = DriverStation.getAlliance();

    if (allianceColor.isPresent()) {

      targetTagID = (allianceColor.get().equals(Alliance.Red)) ? 5 : 6;

    } else {

      exitCommand = true;

    }

    // Set setpoint to center the robot on the amp in the x direction
    xController.setSetpoint(VisionConstants.AMP_YAW_ANGLE);

    // Set setpoint to drive the robot up against the amp
    yController.setSetpoint(VisionConstants.AMP_PITCH_ANGLE);

    // Set setpoint to turn the robot to the correct angle (degrees to work with the
    // drive request)
    if (crescendoField.getTagPose(targetTagID).isPresent()) {

      // TODO check invert
      targetRotation = crescendoField.getTagPose(targetTagID).get().getRotation().toRotation2d();

    } else {

      exitCommand = true;

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentX = xFilter.calculate(Cameras.getYaw(Cameras.ampCamera, targetTagID));
    double currentY = yFilter.calculate(Cameras.getPitch(Cameras.ampCamera, targetTagID));

    // Ends command if no AprilTag is detected in the camera frame
    // Camera methods return 180.0 if the target tag ID is not detected
    if (currentX == 180.0 || currentY == 180.0) {

      exitCommand = true;

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

    SmartDashboard.putNumber("Target X (Yaw)", xController.getSetpoint());
    SmartDashboard.putNumber("Target Y (Pitch)", yController.getSetpoint());
    SmartDashboard.putNumber("Target Rotation (Radians)", visionDriveRequest.HeadingController.getSetpoint());

    SmartDashboard.putNumber("Current X (Yaw)", currentX);
    SmartDashboard.putNumber("Current Y (Pitch)", currentY);
    SmartDashboard.putNumber("Current Rotation (Radians)", Units.degreesToRadians(driveSubsystem.getPigeon2().getAngle()));

    SmartDashboard.putNumber("X Speed", xSpeed);
    SmartDashboard.putNumber("Y Speed", ySpeed);

    // TODO check inverts on Frog, both alliances
    driveSubsystem.setControl(
        visionDriveRequest.withVelocityX(xSpeed)
            .withVelocityY(-ySpeed)
            .withTargetDirection(targetRotation));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (exitCommand) {

      System.out.println("*****AmpVisionCommand Interrupted*****");
      System.out.println("Alliance Present: " + allianceColor.isPresent());
      System.out.println("AprilTag Pose Present: " + crescendoField.getTagPose(targetTagID).isPresent());

    }

    driveSubsystem.setControl(
        visionDriveRequest.withVelocityX(0)
            .withVelocityY(0)
            .withTargetDirection(driveSubsystem.getPigeon2().getRotation2d()));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xController.atSetpoint() && yController.atSetpoint() && visionDriveRequest.HeadingController.atSetpoint())
        || exitCommand;
  }
}
