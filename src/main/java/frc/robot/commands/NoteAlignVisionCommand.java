// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class NoteAlignVisionCommand extends Command {
  DriveSubsystem driveSubsystem;

  //TODO update camera name
  PhotonCamera camera7;

  //TODO update max velocity and acceleration
  private static final double maxVelocity = 2;
  private static final double maxAcceleration = 2;

  private static final double maxAngularVelocity = 2 * Math.PI;
  private static final double maxAngularAcceleration = Math.PI;

  //TODO update PID values
  private static final double xP = 0;
  private static final double xI = 0;
  private static final double xD = 0;

  private static final double rotationP = 0;
  private static final double rotationI = 0;
  private static final double rotationD = 0;

  private static final TrapezoidProfile.Constraints linearConstraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
  private static final TrapezoidProfile.Constraints angularConstraints = new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration);

  private static final ProfiledPIDController xController = new ProfiledPIDController(xP, xI, xD, linearConstraints);
  private static final ProfiledPIDController rotationController = new ProfiledPIDController(rotationP, rotationI, rotationD, angularConstraints);

  private final SwerveRequest.RobotCentric driveRequest;

  /** Creates a new NoteAlignVisionCommand. */
  public NoteAlignVisionCommand(DriveSubsystem drive) {
    driveSubsystem = drive;

    camera7 = new PhotonCamera("Camera 7");

    driveRequest = new SwerveRequest.RobotCentric();
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult noteResult = camera7.getLatestResult();

    if(noteResult.hasTargets()) {

      PhotonTrackedTarget noteTarget = noteResult.getBestTarget();

      rotationController.setGoal(0);
      //TODO update target yaw
      xController.setGoal(0);

      // Left yaw is negative, right yaw is positive
      // Down pitch is negative, up pitch is positive

      // Running these separately because it is set to robot relative drive and we don't want it to arch
      // around the note
      //TODO test to see if the X and rotation can be run at the same time
      if(!rotationController.atSetpoint()) {
        //TODO determine what units the calculate() outputs, convert to m/s if necessary
        driveSubsystem.setControl(
          driveRequest.withVelocityX(0)
          .withVelocityY(0)
          .withRotationalRate(rotationController.calculate(noteTarget.getYaw())));
      } else {
        driveSubsystem.setControl(
          driveRequest.withVelocityX(xController.calculate(noteTarget.getPitch()))
          .withVelocityY(0)
          .withRotationalRate(0));
      }

    }
    
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
    //TODO change to when a TOF sensor in transition is triggered
    return false;
  }
}
