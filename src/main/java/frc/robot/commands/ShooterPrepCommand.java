// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem.AngleConstants;
import frc.robot.subsystems.ShooterSubsystem.ShooterConstants;

public class ShooterPrepCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ShooterAngleSubsystem angleSubsystem;

  // In meters, represents distances spaced in 1 ft intervals
  // Starting at 3 ft from subwoofer to front of bumper
  private static double[] distanceArray = {
      0.9144 + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH,
      1.2192 + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH,
      1.5240 + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH,
      1.8288 + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH,
      2.1336 + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH,
      2.4384 + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH,
      2.7432 + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH,
      3.0480 + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH,
      3.3528 + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH,
      3.6576 + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH,
      3.9624 + TrajectoryConstants.CENTER_OF_ROBOT_LENGTH
  };

  private double distanceToSpeaker;

  private Pose2d targetPose;
  private Pose2d currentPose;

  private double bottomShooterRPM;
  private double topShooterOffset = -100;
  private double shooterPivotAngle;

  private static final double shooterAllowableError = 100;
  private static final double angleAllowableError = 0.1;

  private Timer time;

  /** Creates a new ShooterPrepCommand. */
  public ShooterPrepCommand(DriveSubsystem drive, ShooterSubsystem shooter, ShooterAngleSubsystem angle) {
    driveSubsystem = drive;
    shooterSubsystem = shooter;
    angleSubsystem = angle;

    time = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, shooterSubsystem, angleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Set shooter to slower idle speed
    shooterSubsystem.setTopShooterMotorVelocity(ShooterConstants.SHOOTER_IDLE_RPM);
    shooterSubsystem.setBottomShooterMotorVelocity(ShooterConstants.SHOOTER_IDLE_RPM);

    time.restart();

    Optional<DriverStation.Alliance> allianceColor = DriverStation.getAlliance();

    if (allianceColor.isPresent()) {
      if (allianceColor.get().equals(Alliance.Red)) {

        targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(4).get().toPose2d();

      } else if (allianceColor.get().equals(Alliance.Blue)) {

        targetPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(7).get().toPose2d();

      }
    } else {

      targetPose = driveSubsystem.getCurrentPose2d();

    }

    currentPose = driveSubsystem.getCurrentPose2d();

    distanceToSpeaker = Math.hypot((currentPose.getX() - targetPose.getX()),
        (currentPose.getY() - targetPose.getY()));

    if (distanceToSpeaker > distanceArray[10]) {

      bottomShooterRPM = ShooterConstants.SHOOTER_RPM[10];

      shooterPivotAngle = AngleConstants.PIVOT_ANGLE[10];

    } else {

      for (int index = 0; index < distanceArray.length; index++) {

        if (distanceToSpeaker < distanceArray[index]) {

          bottomShooterRPM = ShooterConstants.SHOOTER_RPM[index];

          double percentDifference = (distanceArray[index + 1] - distanceToSpeaker) / Units.feetToMeters(1);
          double angleAdjust = percentDifference
              * (AngleConstants.PIVOT_ANGLE[index] - AngleConstants.PIVOT_ANGLE[index + 1]);

          shooterPivotAngle = AngleConstants.PIVOT_ANGLE[index + 1] + angleAdjust;

          break;

        }

      }

    }

    shooterSubsystem.setBottomShooterMotorVelocity(bottomShooterRPM);
    shooterSubsystem.setTopShooterMotorVelocity(bottomShooterRPM + topShooterOffset);

    angleSubsystem.setAngle(shooterPivotAngle);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    time.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (shooterSubsystem.getBottomMotorVelocity() > (bottomShooterRPM - shooterAllowableError))
        && (shooterSubsystem.getTopMotorVelocity() > (bottomShooterRPM + topShooterOffset - shooterAllowableError))
        && (Math.abs(angleSubsystem.getAngleEncoderPosition() - shooterPivotAngle) < angleAllowableError)
        || time.hasElapsed(0.5);
  }
}
