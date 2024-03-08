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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Cameras;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveConstants;
import frc.robot.subsystems.LEDSubsystem.LEDConstants.LEDColors;

public class DriveCommand extends Command {
  private DriveSubsystem driveSubsystem;

  private double percentDeadband = 0.1;

  private double maxSpeed = DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC;
  private double maxAngularRate = 2 * Math.PI;

  // In radians (using camera degrees)
  private PIDController xController = new PIDController(2.31, 0, 0);
  private PIDController yController = new PIDController(2.31, 0, 0);

  //TODO tune these error ranges
  private double allowableXError = 0.035;
  private double allowableYError = 0.035;

  private double currentX = 0;
  private double currentY = 0;

  // In radians
  private PIDController rotationController = new PIDController(0.1, 0, 0);
  private double allowableRotationError = 0.035;
  private double currentRotation = 0;

  private final CommandXboxController driverController;


  private final SwerveRequest.FieldCentric driveRequest; 
  private final SwerveRequest.FieldCentricFacingAngle visionDriveRequest;

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

    visionDriveRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    xDriveSlew = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_RATE);
    yDriveSlew = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_RATE);

    xController.setTolerance(allowableXError);
    yController.setTolerance(allowableYError);
    rotationController.setTolerance(allowableRotationError);

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

    invert = (allianceColor.isPresent() && allianceColor.get().equals(Alliance.Red)) ? -1 : 1; 

    double xSpeed = 0;
    double ySpeed = 0;
    double rotationalSpeed = 0;

    //TODO update to use correct button
    if(driverController.a().getAsBoolean()) {

      LEDSubsystem.setColor(LEDColors.BLUE);

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

          //TODO remove after testing
          SmartDashboard.putNumber("X Setpoint", xController.getSetpoint());
          SmartDashboard.putNumber("Y Setpoint", yController.getSetpoint());
          SmartDashboard.putNumber("Rotation Setpoint", rotationController.getSetpoint());

          currentX = Cameras.getYaw(Cameras.ampCamera, targetTagID);
          currentY = Cameras.getPitch(Cameras.ampCamera, targetTagID);
          currentRotation = Units.degreesToRadians(driveSubsystem.getPigeon2().getAngle());

          SmartDashboard.putNumber("Current x", currentX);
          SmartDashboard.putNumber("Current y", currentY);
          SmartDashboard.putNumber("Current rotation", currentRotation);

          xSpeed = xController.calculate(currentX);
          ySpeed = yController.calculate(currentY);
          rotationalSpeed = rotationController.calculate(currentRotation);
          
          break;

        case SPEAKER:

          if(allianceColor.isPresent()) {

            targetTagID = allianceColor.get().equals(Alliance.Red) ? 4 : 7;

          }


          // Check distance from the target using camera pitch
          for(int i = 0; i < VisionConstants.SPEAKER_PITCH_ARRAY.length; i++) {

            if(Cameras.getPitch(Cameras.speakerCamera, targetTagID) >= VisionConstants.SPEAKER_PITCH_ARRAY[i]) {

              // Set rotation to turn to center on the speaker
              rotationController.setSetpoint(VisionConstants.SPEAKER_YAW_ARRAY[i]);

              break;

            }

          }

          SmartDashboard.putNumber("Rotation Setpoint", rotationController.getSetpoint());

          rotationalSpeed = rotationController.calculate(Cameras.getYaw(Cameras.speakerCamera, targetTagID));

          break;

        case TRAP:

          break;

        case SUB_SHOOT, SUB_PLUS_ROBOT_SHOOT, DRIVE, INTAKE, BARF, CLIMB:

          break;

        default:

          break;
      }

      if(xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint()) {
        driveSubsystem.setControl(
          visionDriveRequest.withVelocityX(0)
          .withVelocityY(0)
          .withTargetDirection(new Rotation2d(driveSubsystem.getPigeon2().getAngle()))
        );
      } else {

        driveSubsystem.setControl(
          visionDriveRequest.withVelocityX(xSpeed)
          .withVelocityY(ySpeed * invert));

      }
      

    } else {

      xSpeed = xDriveSlew.calculate(-driverController.getLeftY() * DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC);
      ySpeed = yDriveSlew.calculate(-driverController.getLeftX() * DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC);
      rotationalSpeed = -driverController.getRightX() * DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC;

      driveSubsystem.setControl(
        driveRequest.withVelocityX(xSpeed * invert)
        .withVelocityY(ySpeed * invert)
        .withRotationalRate(rotationalSpeed));

    }

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rotationalSpeed", rotationalSpeed);

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