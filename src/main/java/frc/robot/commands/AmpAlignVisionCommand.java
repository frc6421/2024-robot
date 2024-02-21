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

public class AmpAlignVisionCommand extends Command {
  DriveSubsystem driveSubsystem;

  //TODO update max velocity and acceleration
  // In m/s
  private static final double maxLinearVelocity = 2;
  private static final double maxLinearAcceleration = 2;

  //TODO update max velocity and acceleration
  // In radians/sec
  private static final double maxAngularVelocity = 2 * Math.PI;
  private static final double maxAngularAcceleration = Math.PI;

  private Pose2d targetPose;
  private Pose2d currentPose;

  // In meters
  private double allowableXError = 0.02;
  // In meters
  private double allowableYError = 0.02;
  // In radians
  private double allowableAngleError = 0.04;

  //TODO update PID values
  private static final double xP = 0;
  private static final double xI = 0;
  private static final double xD = 0;

  private static final double yP = 0;
  private static final double yI = 0;
  private static final double yD = 0;

  //TODO update PID values
  private static final double rotationP = 0;
  private static final double rotationI = 0;
  private static final double rotationD = 0;

  private static final TrapezoidProfile.Constraints linearConstraints = new TrapezoidProfile.Constraints(maxLinearVelocity, maxLinearAcceleration);
  private static final TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration);

  private static final ProfiledPIDController xController = new ProfiledPIDController(xP, xI, xD, linearConstraints);
  private static final ProfiledPIDController yController = new ProfiledPIDController(yP, yI, yD, linearConstraints);

  private static final ProfiledPIDController rotationController = new ProfiledPIDController(rotationP, rotationI, rotationD, rotationConstraints);

  private final SwerveRequest.FieldCentric driveRequest;
  
  /** Creates a new AmpAlignVisionCommand. */
  public AmpAlignVisionCommand(DriveSubsystem drive) {
    driveSubsystem = drive;

    driveRequest = new SwerveRequest.FieldCentric();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setTolerance(allowableXError);
    yController.setTolerance(allowableYError);
    rotationController.setTolerance(allowableAngleError);

    Optional<DriverStation.Alliance> allianceColor = DriverStation.getAlliance();

    if(allianceColor.get().equals(Alliance.Red)) {
      targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(5).get().toPose2d();
    } else if(allianceColor.get().equals(Alliance.Blue)) {
      targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(6).get().toPose2d();
    }

    //TODO add/subtract robot size if needed
    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());
    rotationController.setGoal(targetPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPose = driveSubsystem.getCurrentPose2d();

    driveSubsystem.setControl(driveRequest
      .withVelocityX(xController.calculate(currentPose.getX()))
      .withVelocityY(yController.calculate(currentPose.getY()))
      .withRotationalRate(rotationController.calculate(currentPose.getRotation().getRadians())));
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setControl(driveRequest
      .withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint();
  }
}