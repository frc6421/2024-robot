// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Cameras;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveConstants;

public class DriveCommand extends Command {
  private DriveSubsystem driveSubsystem;

  private double percentDeadband = 0.1;

  private double maxSpeed = DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC;
  private double maxAngularRate = 2 * Math.PI;

  // In meters
  private PIDController xController = new PIDController(2.31, 0, 0);
  private PIDController yController = new PIDController(2.31, 0, 0);

  private double allowableXError = 0.02;
  private double allowableYError = 0.02;

  private double targetX = 0;
  private double targetY = 0;

  // In radians
  private PIDController rotationController = new PIDController(5, 0, 0);
  private double allowableAngleError = 0.035;
  private double targetAngle = 0;

  private final CommandXboxController driverController;


  private final SwerveRequest.FieldCentric driveRequest; 

  private final AprilTagFieldLayout crescendoField = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private int targetTagID = 0;

  private final SlewRateLimiter xDriveSlew;
  private final SlewRateLimiter yDriveSlew;

  private int invert = 1;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem drive, CommandXboxController controller) {

    driveSubsystem = drive;
    driverController = controller;
   

    driveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(maxSpeed * percentDeadband).withRotationalDeadband(maxAngularRate * percentDeadband) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    xDriveSlew = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_RATE);
    yDriveSlew = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_RATE);

    xController.setTolerance(allowableXError);
    yController.setTolerance(allowableYError);
    rotationController.setTolerance(allowableAngleError);

    rotationController.enableContinuousInput(-Math.PI, Math.PI);

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
    Optional<DriverStation.Alliance> allianceColor = DriverStation.getAlliance();

    if(allianceColor.isPresent() && allianceColor.get().equals(Alliance.Red)) {
      invert = -1;
    }

    double xSpeed = 0;
    double ySpeed = 0;
    double rotationalSpeed = 0;

    //TODO update to use correct buttons
    if(driverController.a().getAsBoolean()) {

      switch(RobotContainer.robotState) {
        case AMP:
        
          if(allianceColor.isPresent()) {

            targetTagID = (allianceColor.get().equals(Alliance.Red)) ? 5 : 6;

          }
          
          // Set setpoint to center the robot on the amp in the x direction
          xController.setSetpoint(VisionConstants.AMP_YAW_ANGLE);

          // Set setpoint to drive the robot up against the amp
          yController.setSetpoint(VisionConstants.AMP_PITCH_ANGLE);

          // Set setpoint to turn the robot to the correct angle
          if(crescendoField.getTagPose(targetTagID).isPresent()) {
            rotationController.setSetpoint(crescendoField.getTagPose(targetTagID).get().getRotation().toRotation2d().getRadians());
          }

          xSpeed = xController.calculate(Cameras.getYaw(Cameras.ampCamera, targetTagID));
          ySpeed = yController.calculate(Cameras.getPitch(Cameras.ampCamera, targetTagID));
          rotationalSpeed = rotationController.calculate(Units.degreesToRadians(driveSubsystem.getPigeon2().getAngle()));
          
          break;

        case SPEAKER:

          if(allianceColor.isPresent()) {

            targetTagID = allianceColor.get().equals(Alliance.Red) ? 4 : 7;

          }


          for(int i = 0; i < VisionConstants.SPEAKER_PITCH_ARRAY.length; i++) {

            if(Cameras.getPitch(Cameras.speakerCamera, targetTagID) >= VisionConstants.SPEAKER_PITCH_ARRAY[i]) {

              rotationController.setSetpoint(VisionConstants.SPEAKER_YAW_ARRAY[i]);

              break;

            }

          }

            rotationalSpeed = rotationController.calculate(Cameras.getYaw(Cameras.speakerCamera, targetTagID));

          break;

        case TRAP:

          break;

        case SUB_SHOOT, SUB_PLUS_ROBOT_SHOOT, DRIVE, INTAKE, BARF, CLIMB:

          break;

        default:

          break;
      }


    } else {

      xSpeed = xDriveSlew.calculate(-driverController.getLeftY() * DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC);
      ySpeed = yDriveSlew.calculate(-driverController.getLeftX() * DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC);
      rotationalSpeed = -driverController.getRightX() * DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC;

    }

    driveSubsystem.setControl(
        driveRequest.withVelocityX(xSpeed * invert)
        .withVelocityY(ySpeed * invert)
        .withRotationalRate(rotationalSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

}