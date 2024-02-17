// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedFourPieceCommand extends SequentialCommandGroup {
  private DriveSubsystem driveSubsystem;
  private IntakeSubsystem intakeSubsystem;

   private Field2d field;
  /** Creates a new BlueTwoPieceCommand. */
  public RedFourPieceCommand(DriveSubsystem drive, IntakeSubsystem intake) {

    driveSubsystem = drive;
    intakeSubsystem = intake;
    addRequirements(driveSubsystem, intakeSubsystem);

    TrajectoryConfig forwardConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND - 3.5,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED - 4)
        .setKinematics(driveSubsystem.kinematics);
    
    TrajectoryConfig reverseConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND + 0.5,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED + 1.5)
        .setKinematics(driveSubsystem.kinematics)
        .setReversed(true);

    // robot leaves start zone and moves to pick up note at podium
    Trajectory driveToFirstNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.FRONT_CENTER_RED_SUBWOOFER, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.NOTE9, new Rotation2d(0))), reverseConfig);

    Trajectory driveToSecondNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.NOTE9, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.NOTE10, new Rotation2d(0))), reverseConfig);

    Trajectory driveToThirdNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.NOTE10, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.NOTE11, new Rotation2d(0))), reverseConfig);

    // Simulation
     field = new Field2d();

     if (RobotBase.isSimulation()) {
        SmartDashboard.putData(field);

        field.setRobotPose(driveToFirstNoteTrajectory.getInitialPose());
      
        field.getObject("Drive to first Trajectory").setTrajectory(driveToFirstNoteTrajectory);
        field.getObject("Drive to second Trajectory").setTrajectory(driveToSecondNoteTrajectory);
        field.getObject("Drive to third Trajectory").setTrajectory(driveToThirdNoteTrajectory);
      }

    var thetaController = new ProfiledPIDController(
        AutoConstants.THETA_P, AutoConstants.THETA_I, AutoConstants.THETA_D,
        new TrapezoidProfile.Constraints(AutoConstants.AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
            AutoConstants.AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
        // Position controllers
        new PIDController(AutoConstants.X_DRIVE_P, AutoConstants.X_DRIVE_I, AutoConstants.X_DRIVE_D),
        new PIDController(AutoConstants.Y_DRIVE_P, AutoConstants.Y_DRIVE_I, AutoConstants.Y_DRIVE_D),
        thetaController);

    SwerveControllerCommand driveToFirstNoteCommand = new SwerveControllerCommand(
        driveToFirstNoteTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.kinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    SwerveControllerCommand driveToSecondNoteCommand = new SwerveControllerCommand(
        driveToSecondNoteTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.kinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    SwerveControllerCommand driveToThirdNoteCommand = new SwerveControllerCommand(
        driveToThirdNoteTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.kinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> driveSubsystem.tareEverything()), 
      // score pre-loaded piece 
      new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_IN_SPEED)), 
      driveToFirstNoteCommand,
      new InstantCommand(() -> driveSubsystem.setControl(new SwerveRequest.ApplyChassisSpeeds())),
      // stop intake and score second piece
      driveToSecondNoteCommand,
      driveToThirdNoteCommand
    );
  }
}
