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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Cameras;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TrapVisionCommand extends Command {
  private DriveSubsystem driveSubsystem;

  // In meters
  private double maxSpeed = 2;

  // In radians
  private double maxAngularVelocity = 2 * Math.PI;

  // In camera degrees
  // TODO tune PID
  private PIDController xController;
  private PIDController yController;
  private PIDController rotationController;

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
  private double rotationalSpeed = 0.0;

  private final SwerveRequest.RobotCentric visionDriveRequest;

  private MedianFilter xFilter = new MedianFilter(10);
  private MedianFilter yFilter = new MedianFilter(10);

  private Optional<DriverStation.Alliance> allianceColor;

  private final AprilTagFieldLayout crescendoField = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private int targetTagID = 0;

  private boolean exitCommand = false;

  /** Creates a new TrapVisionCommand. */
  public TrapVisionCommand(DriveSubsystem drive) {
    driveSubsystem = drive;

    visionDriveRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    xController = new PIDController(xP, 0.0, 0.0);
    yController = new PIDController(yP, 0.0, 0.0);
    rotationController = new PIDController(rotationP, 0.0, 0.0);

    xController.setTolerance(allowableXError);
    yController.setTolerance(allowableYError);
    rotationController.setTolerance(allowableRotationError);

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
    rotationController.reset();

    allianceColor = DriverStation.getAlliance();

    if (allianceColor.isPresent()) {

      for (int i = 0; i < Cameras.getListOfTargets(Cameras.ampCamera).length; i++) {

        if (Cameras.getTagIDFromList(Cameras.ampCamera, i) == 11) {
          // Red left stage
          targetTagID = 11;

        } else if (Cameras.getTagIDFromList(Cameras.ampCamera, i) == 12) {
          // Red right stage
          targetTagID = 12;

        } else if (Cameras.getTagIDFromList(Cameras.ampCamera, i) == 13) {
          // Red center stage
          targetTagID = 13;

        } else if (Cameras.getTagIDFromList(Cameras.ampCamera, i) == 14) {
          // Blue center stage
          targetTagID = 14;

        } else if (Cameras.getTagIDFromList(Cameras.ampCamera, i) == 15) {
          // Blue left stage
          targetTagID = 15;

        } else if (Cameras.getTagIDFromList(Cameras.ampCamera, i) == 16) {
          // Blue right stage
          targetTagID = 16;

        }

      }

    } else {

      exitCommand = true;

    }

    // Set setpoint to center the robot on the amp in the x direction (robot
    // relative)
    xController.setSetpoint(VisionConstants.TRAP_PITCH_ANGLE);

    // Set setpoint to drive the robot to a set distance from the trap tag (robot
    // relative)
    yController.setSetpoint(VisionConstants.TRAP_YAW_ANGLE);

    // Set setpoint to turn the robot to the correct angle (radians)
    if (crescendoField.getTagPose(targetTagID).isPresent()) {

      rotationController.setSetpoint(crescendoField.getTagPose(targetTagID).get().getRotation().toRotation2d().getRadians());

    } else {

      exitCommand = true;

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentX = xFilter.calculate(Cameras.getYaw(Cameras.ampCamera, targetTagID));
    double currentY = yFilter.calculate(Cameras.getPitch(Cameras.ampCamera, targetTagID));
    double currentRotation = driveSubsystem.getPigeon2().getRotation2d().getRadians();

    // Ends command if no AprilTag is detected in the camera frame
    // Camera methods return 180.0 if the target tag ID is not detected
    if (currentX == 180.0 || currentY == 180.0) {

      exitCommand = true;

    } else {

      if(xController.atSetpoint()) {
         
        xSpeed = 0.0;

      } else {

        xSpeed = xController.calculate(currentX);

      }
      
      if(yController.atSetpoint()) {

        ySpeed = 0.0;

      } else {

        ySpeed = yController.calculate(currentY);

      }
      
      rotationalSpeed = rotationController.calculate(currentRotation);

    }

    // SmartDashboard.putNumber("Target X (Yaw)", xController.getSetpoint());
    // SmartDashboard.putNumber("Target Y (Pitch)", yController.getSetpoint());
    // SmartDashboard.putNumber("Target Rotation (Radians)", rotationController.getSetpoint());

    // SmartDashboard.putNumber("Current X (Yaw)", currentX);
    // SmartDashboard.putNumber("Current Y (Pitch)", currentY);
    // SmartDashboard.putNumber("Current Rotation (Radians)", currentRotation);

    driveSubsystem.setControl(
        visionDriveRequest.withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(rotationalSpeed));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (exitCommand) {

      System.out.println("*****TrapVisionCommand Interrupted*****");

    }

    driveSubsystem.setControl(
        visionDriveRequest.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint()) || exitCommand;
  }
}
