// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tuning;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.PointWheelsAt;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class StaticFeedforwardCommand extends Command {
  private final DriveSubsystem driveSubsystem;

  private final ChassisSpeeds velocityDelta;
  private ChassisSpeeds setChassisSpeeds;

  private final SwerveRequest.PointWheelsAt zeroWheelRequest;
  private final SwerveRequest.ApplyChassisSpeeds stopRobotRequest;
  private final SwerveRequest.ApplyChassisSpeeds driveByChassisSpeedsRequest;

  /**
   * Distance used to determine if the robot is moving.
   * </p>
   * *** May want to change value depending on data output.
   */
  private final static double ROBOT_IS_MOVING_METERS = 0.01;

  /**
   * Used to determine the feedforward voltage needed to just get the robot to
   * move.
   * </p>
   * When done update the kS value of the Drive Motor
   * {@link DriveSubsystem#DRIVE_GAINS}.
   * </p>
   * *** May need to be updated if the weight of the robot changes significantly.
   * 
   * @param drive the drive subsystem used with command.
   */
  public StaticFeedforwardCommand(DriveSubsystem drive) {
    driveSubsystem = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    // The change in velocity to add each robot cycle.
    velocityDelta = new ChassisSpeeds(0.001, 0, 0);

    // Swerve request to make sure the wheels point in the x direction
    zeroWheelRequest = new PointWheelsAt();
    // Swerve request to stop the robot
    stopRobotRequest = new ApplyChassisSpeeds();
    // Swerve request to use to drive the robot
    driveByChassisSpeedsRequest = new ApplyChassisSpeeds();

    Shuffleboard.getTab("3: Static Feedforward").add(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set wheels to point forward.
    driveSubsystem.setControl(zeroWheelRequest);
    // Set chassis speed to zero each time the command starts
    setChassisSpeeds = new ChassisSpeeds();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive forward at set Chassis Speed
    driveSubsystem.setControl(driveByChassisSpeedsRequest.withSpeeds(setChassisSpeeds));
    System.out.println("Velocity: " + setChassisSpeeds.vxMetersPerSecond);
    // Increase chassis speed.
    setChassisSpeeds = setChassisSpeeds.plus(velocityDelta);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Print the voltage of each drive motor module.
    for (int i = 0; i < 4; ++i) {
      System.out.println("Static Feedforward Voltage Module " + i + ": "
          + driveSubsystem.getModule(i).getDriveMotor().getMotorVoltage().getValue());
    }
    // Stop the robot.
    driveSubsystem.setControl(stopRobotRequest);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Return true of robot moves 1 cm
    return driveSubsystem.getState().Pose.getX() >= ROBOT_IS_MOVING_METERS;
  }
}
