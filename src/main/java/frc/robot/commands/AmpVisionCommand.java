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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
  private double maxSpeed = 3;
  private double maxAcceleration = 3;

  private TrapezoidProfile.Constraints linearConstraints = new Constraints(maxSpeed, maxAcceleration);

  // In camera degrees
  // TODO tune PID
  private ProfiledPIDController xController;
  private ProfiledPIDController yController;

  private double xP = 2.31;
  private double yP = 2.31;
  private double rotationP = 1.0;

  // TODO tune these error ranges
  // In camera degrees
  private double allowableXError = 0.5;
  private double allowableYError = 0.5;

  // In gyro degrees
  private double allowableRotationError = 2;

  private double xSpeed = 0.0;
  private double ySpeed = 0.0;

  private Rotation2d targetRotation;

  private final SwerveRequest.FieldCentricFacingAngle visionDriveRequest;

  private Optional<DriverStation.Alliance> allianceColor;

  private final AprilTagFieldLayout crescendoField = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private int targetTagID = 0;

  /** Creates a new AmpTrapVisionCommand. */
  public AmpVisionCommand(DriveSubsystem drive) {
    driveSubsystem = drive;

    visionDriveRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    visionDriveRequest.HeadingController.setPID(rotationP, 0.0, 0.0);
    visionDriveRequest.HeadingController.setTolerance(allowableRotationError);
    visionDriveRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    xController = new ProfiledPIDController(xP, 0.0, 0.0, linearConstraints);
    yController = new ProfiledPIDController(yP, 0.0, 0.0, linearConstraints);

    xController.setTolerance(allowableXError);
    yController.setTolerance(allowableYError);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    allianceColor = DriverStation.getAlliance();

    if (allianceColor.isPresent()) {

      targetTagID = (allianceColor.get().equals(Alliance.Red)) ? 5 : 6;

    } else {

      end(true);

    }

    // Set setpoint to center the robot on the amp in the x direction
    xController.setGoal(VisionConstants.AMP_YAW_ANGLE);

    // Set setpoint to drive the robot up against the amp
    yController.setGoal(VisionConstants.AMP_PITCH_ANGLE);

    // Set setpoint to turn the robot to the correct angle (degrees to work with the drive request)
    if (crescendoField.getTagPose(targetTagID).isPresent()) {

      targetRotation = crescendoField.getTagPose(targetTagID).get().getRotation().toRotation2d();

    } else {

      end(true);

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentX = Cameras.getYaw(Cameras.ampCamera, targetTagID);
    double currentY = Cameras.getPitch(Cameras.ampCamera, targetTagID);

    if (RobotContainer.robotState.equals(RobotStates.AMP)) {

      // Ends command if no AprilTag is detected in the camera frame
      // Camera methods return 180.0 if the target tag ID is not detected
      if (currentX == 180.0 || currentY == 180.0) {

        end(true);

      } else {

        xSpeed = xController.calculate(currentX);
        ySpeed = yController.calculate(currentY);

      }

    } else {

      end(true);

    }

    SmartDashboard.putNumber("Target X (Yaw)", xController.getGoal().position);
    SmartDashboard.putNumber("Target Y (Pitch)", yController.getGoal().position);
    SmartDashboard.putNumber("Target Rotation (Degrees)", visionDriveRequest.HeadingController.getSetpoint());

    SmartDashboard.putNumber("Current X (Yaw)", currentX);
    SmartDashboard.putNumber("Current Y (Pitch)", currentY);
    SmartDashboard.putNumber("Current Rotation (Degrees)", driveSubsystem.getPigeon2().getAngle());

    driveSubsystem.setControl(
        visionDriveRequest.withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withTargetDirection(targetRotation));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (interrupted) {

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
    return xController.atGoal() && yController.atGoal() && visionDriveRequest.HeadingController.atSetpoint();
  }
}
