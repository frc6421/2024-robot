// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tuning;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.PointWheelsAt;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveConstants;

public class TuneCurrentLimitCommand extends Command {

  private final DriveSubsystem driveSubsystem;

  private final ChassisSpeeds velocityDelta;
  private ChassisSpeeds setChassisSpeeds;

  private final PointWheelsAt zeroWheelRequest;
  private final ApplyChassisSpeeds stopRobotRequest;
  private final ApplyChassisSpeeds driveByChassisSpeedsRequest;

  // TODO determine the velocity to set which means the wheels are slipping.
  /** In meters per second */
  private static final double VELOCITY_LIMIT = 0.1;

  /**
   * Used to find the current at which wheels start slipping.
   * </p>
   * To use place robot against wall or other unmovable barrier and start command.
   * </p>
   * When done update the value in {@link DriveSubsystem#SLIP_CURRENT_AMPS}
   * </p>
   * *** May need to be updated as treads wear and/or are replaced.
   * 
   * @param drive the drive subsystem used with command.
   */
  public TuneCurrentLimitCommand(DriveSubsystem drive) {
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

    // Add command to shuffleboard.
    Shuffleboard.getTab("2: Current Limits").add(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set chassis speeds to 0 each time command starts
    setChassisSpeeds = new ChassisSpeeds();
    // Make sure wheels are pointed forward.
    driveSubsystem.setControl(zeroWheelRequest.withModuleDirection(new Rotation2d()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Start driveing forward
    driveSubsystem.setControl(driveByChassisSpeedsRequest.withSpeeds(setChassisSpeeds));
    System.out.println("X velocity: " + setChassisSpeeds.vxMetersPerSecond);

    // Add to the X velocity.
    setChassisSpeeds = setChassisSpeeds.plus(velocityDelta);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Print stator and supply currents for each module
    for (int i = 0; i < 4; ++i) {
      System.out.println("Module " + i + " Stator Current: "
          + driveSubsystem.getModule(i).getDriveMotor().getStatorCurrent().getValue());
      System.out.println("Module " + i + " Supply Current: "
          + driveSubsystem.getModule(i).getDriveMotor().getSupplyCurrent().getValue());
    }
    // Stop the motors.
    driveSubsystem.setControl(stopRobotRequest);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stop command if one wheel is slipping
    return isWheelSlipping();
  }

  /**
   * Determine if one wheel is slipping if the velocity is higher than value.
   * 
   * @return
   */
  private boolean isWheelSlipping() {
    // Cycle through all module drive motors to see if one is slipping.
    // Get velocity is in rotations per second, convert to meters per secord for
    // better understanding.
    for (int i = 0; i < 4; ++i) {
      if (driveSubsystem.getModule(i).getDriveMotor().getVelocity().getValue()
          / DriveConstants.DRIVE_ROTATIONS_PER_METER > VELOCITY_LIMIT)
        return true;
    }
    return false;
  }
}
