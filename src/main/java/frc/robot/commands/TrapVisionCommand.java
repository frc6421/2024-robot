// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
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

public class TrapVisionCommand extends Command {
  private DriveSubsystem driveSubsystem;

  // In meters
  private double maxSpeed = 3;
  private double maxAcceleration = 3;

  // In radians
  private double maxAngularVelocity = 2 * Math.PI;
  private double maxAngularAcceleration = 2 * Math.PI;

  private TrapezoidProfile.Constraints linearConstraints = new Constraints(maxSpeed, maxAcceleration);
  private TrapezoidProfile.Constraints angularConstraints = new Constraints(maxAngularVelocity, maxAngularAcceleration);

  // In camera degrees
  // TODO tune PID
  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private ProfiledPIDController rotationController;

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
  private double rotationalSpeed = 0.0;

  private final SwerveRequest.RobotCentric visionDriveRequest;

  private Optional<DriverStation.Alliance> allianceColor;

  private final AprilTagFieldLayout crescendoField = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private int targetTagID = 0;

  /** Creates a new TrapVisionCommand. */
  public TrapVisionCommand(DriveSubsystem drive) {
    driveSubsystem = drive;

    visionDriveRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    xController = new ProfiledPIDController(xP, 0.0, 0.0, linearConstraints);
    yController = new ProfiledPIDController(yP, 0.0, 0.0, linearConstraints);
    rotationController = new ProfiledPIDController(rotationP, 0.0, 0.0, angularConstraints);

    xController.setTolerance(allowableXError);
    yController.setTolerance(allowableYError);
    rotationController.setTolerance(allowableRotationError);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    allianceColor = DriverStation.getAlliance();

    if (allianceColor.isPresent()) {

      for(int i = 0; i < Cameras.getListOfTargets(Cameras.ampCamera).length; i++) {

        if(Cameras.getTagIDFromList(Cameras.ampCamera, i) == 11) {
          // Red left stage
          targetTagID = 11;

        } else if(Cameras.getTagIDFromList(Cameras.ampCamera, i) == 12) {
          // Red right stage
          targetTagID = 12;

        } else if(Cameras.getTagIDFromList(Cameras.ampCamera, i) == 13) {
          // Red center stage
          targetTagID = 13;

        } else if(Cameras.getTagIDFromList(Cameras.ampCamera, i) == 14) {
          // Blue center stage
          targetTagID = 14;

        } else if(Cameras.getTagIDFromList(Cameras.ampCamera, i) == 15) {
          // Blue left stage
          targetTagID = 15;

        } else if(Cameras.getTagIDFromList(Cameras.ampCamera, i) == 16) {
          // Blue right stage
          targetTagID = 16;

        }

      }

    } else {

      end(true);

    }

    // Set setpoint to center the robot on the amp in the x direction (robot relative)
    xController.setGoal(VisionConstants.TRAP_YAW_ANGLE);

    // Set setpoint to drive the robot to a set distance from the trap tag (robot relative)
    yController.setGoal(VisionConstants.TRAP_PITCH_ANGLE);

    // Set setpoint to turn the robot to the correct angle (radians)
    if (crescendoField.getTagPose(targetTagID).isPresent()) {

      rotationController.setGoal(crescendoField.getTagPose(targetTagID).get().getRotation().toRotation2d().getRadians());

    } else {

      end(true);

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentX = Cameras.getYaw(Cameras.ampCamera, targetTagID);
    double currentY = Cameras.getPitch(Cameras.ampCamera, targetTagID);
    double currentRotation = driveSubsystem.getPigeon2().getRotation2d().getRadians();

    if (RobotContainer.robotState.equals(RobotStates.TRAP)) {

      // Ends command if no AprilTag is detected in the camera frame
      // Camera methods return 180.0 if the target tag ID is not detected
      if (currentX == 180.0 || currentY == 180.0) {

        end(true);

      } else {

        xSpeed = xController.calculate(currentX);
        ySpeed = yController.calculate(currentY);
        rotationalSpeed = rotationController.calculate(currentRotation);

      }

    } else {

      end(true);

    }

    SmartDashboard.putNumber("Target X (Yaw)", xController.getGoal().position);
    SmartDashboard.putNumber("Target Y (Pitch)", yController.getGoal().position);
    SmartDashboard.putNumber("Target Rotation (Radians)", rotationController.getGoal().position);

    SmartDashboard.putNumber("Current X (Yaw)", currentX);
    SmartDashboard.putNumber("Current Y (Pitch)", currentY);
    SmartDashboard.putNumber("Current Rotation (Radians)", currentRotation);

    driveSubsystem.setControl(
        visionDriveRequest.withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(rotationalSpeed));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (interrupted) {

      System.out.println("*****TrapVisionCommand Interrupted*****");
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
    return xController.atGoal() && yController.atGoal() && rotationController.atGoal();
  }
}
