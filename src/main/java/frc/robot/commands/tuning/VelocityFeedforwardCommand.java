// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tuning;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveConstants;

public class VelocityFeedforwardCommand extends Command {
  private final DriveSubsystem driveSubsystem;

  private ChassisSpeeds setChassisSpeeds;

  private final SwerveRequest.PointWheelsAt zeroWheelsRequest;
  private final SwerveRequest.ApplyChassisSpeeds driveVelocityRequest;
  private final SwerveRequest.ApplyChassisSpeeds stopRobotRequest;

  /**
   * The velocity to drive the robot in this command.
   * </p>
   * Can be positive or negative but should be less than max robot velocity
   */
  private double robotVelocity;
  
  // Used to display the calculated kV values on the Shuffleboard
  private double[] modulekV = {0.0,0.0,0.0,0.0};

  /**
   * Used to determine the feed forward voltage needed to get the robot moving
   * close to the set velocity.
   * </p>
   * This should be run at several different velocities, both froward and
   * backward, to verify that the data appears good. kV should be fairly linear
   * over the range of velocities. These velocities are drive motor velocity in
   * rpm. (?)
   * </p>
   * When done update the {@link DriveSubsystem#kV} value of the Drive Motor
   * {@link DriveSubsystem#DRIVE_GAINS}.
   * 
   * @param drive the drive subsystem used with command.
   */
  public VelocityFeedforwardCommand(DriveSubsystem drive) {
    driveSubsystem = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    robotVelocity = 0; 

    // Swerve request to make sure the wheels point in the x direction
    zeroWheelsRequest = new SwerveRequest.PointWheelsAt();
    // Swerve request to use to drive the robot
    driveVelocityRequest = new SwerveRequest.ApplyChassisSpeeds();
    // Swerve request to stop the robot
    stopRobotRequest = new SwerveRequest.ApplyChassisSpeeds();

    Shuffleboard.getTab("4: Velocity Feedforward").add(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set wheels to face forward.
    driveSubsystem.setControl(zeroWheelsRequest.withModuleDirection(new Rotation2d()));
    // Reset odometry
    driveSubsystem.tareEverything();
    // Set chassis speed based on the velocity set in Shuffleboard
    setChassisSpeeds = new ChassisSpeeds(robotVelocity, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setControl(driveVelocityRequest.withSpeeds(setChassisSpeeds));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for (int i = 0; i < 4; ++i) {
      double voltage = driveSubsystem.getModule(i).getDriveMotor().getMotorVoltage().getValue();
      double velocity = driveSubsystem.getModule(i).getDriveMotor().getVelocity().getValue();
      System.out.println("Module " + i + " Drive Motor Voltage: " + voltage + " Drive Motor Velocity: " + velocity);
      System.out.println("Module " + i + " Drive Motor kV: " + voltage / velocity + " volts / rpm");
      modulekV[i] = voltage / velocity;
      
    }

    System.out.println("Robot X Velocity: " + driveSubsystem.getSwerveDriveKinematics()
        .toChassisSpeeds(driveSubsystem.getState().ModuleStates).vxMetersPerSecond);

    driveSubsystem.setControl(stopRobotRequest.withSpeeds(new ChassisSpeeds(0,0,0)));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stop the command when the robot has travelled at least 3.5 meters.
    return Math.abs(driveSubsystem.getCurrentPose2d().getX()) > 3.5;
  }

  /**
   * Set velocity to run the robot at
   * 
   * @param velocity x velocity of the robot (-
   *                 {@link DriveSubsystem#SPEED_AT_12_VOLTS_METERS_PER_SEC} to
   *                 {@link DriveSubsystem#SPEED_AT_12_VOLTS_METERS_PER_SEC})
   */
  public void setRobotVelocity(double velocity) {
    this.robotVelocity = MathUtil.clamp(velocity, -1 * DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC,
        DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("X Velocity", () -> robotVelocity, this::setRobotVelocity);
    builder.addDoubleArrayProperty("Module kP", () -> modulekV, null);
  }
}
