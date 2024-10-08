// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.IntakeTransitionCommand;
import frc.robot.commands.ShooterRevUpCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.subsystems.TransitionSubsystem.TransitionConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FlipBlueCenterLineFourPieceCommand extends SequentialCommandGroup {
  private DriveSubsystem driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private TransitionSubsystem transitionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private ShooterAngleSubsystem shooterAngleSubsystem;

  private Field2d field;

  /** Creates a new BlueTwoPieceCommand. */
  public FlipBlueCenterLineFourPieceCommand(DriveSubsystem drive, IntakeSubsystem intake, TransitionSubsystem transition, ShooterSubsystem shooter, ShooterAngleSubsystem shooterAngle) {

    driveSubsystem = drive;
    intakeSubsystem = intake;
    transitionSubsystem = transition;
    shooterSubsystem = shooter;
    shooterAngleSubsystem = shooterAngle;
    addRequirements(driveSubsystem, intakeSubsystem, transitionSubsystem ,shooterSubsystem, shooterAngleSubsystem);

    TrajectoryConfig forwardConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND + 0.1,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED - 1)
        .setKinematics(driveSubsystem.kinematics);
    
    TrajectoryConfig reverseConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND + 0.1,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED - 1)
        .setKinematics(driveSubsystem.kinematics)
        .setReversed(true);

    // robot leaves start zone and moves to pick up note at podium

    Trajectory driveToSecondNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.BLUE_FOUR_PIECE_START, new Rotation2d(Units.degreesToRadians(60))),
        new Pose2d(TrajectoryConstants.BLUE_DONT_HIT_WALL.minus(new Translation2d(Units.inchesToMeters(41), Units.inchesToMeters(-15))), new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.NOTE8_BLUE.plus(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(6))), new Rotation2d(Units.degreesToRadians(0)))), forwardConfig);

    Trajectory driveToScoreSecondNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.NOTE8_BLUE.plus(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(6))), new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(TrajectoryConstants.BLUE_CENTER_SCORE, new Rotation2d(Units.degreesToRadians(18)))), reverseConfig);

    Trajectory driveToThirdNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.BLUE_CENTER_SCORE, new Rotation2d(Units.degreesToRadians(18))),
        new Pose2d(TrajectoryConstants.NOTE7_BLUE.plus(new Translation2d(Units.inchesToMeters(9), Units.inchesToMeters(4))), new Rotation2d(Units.degreesToRadians(-30)))), forwardConfig);

    Trajectory driveToScoreThirdNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.NOTE7_BLUE.plus(new Translation2d(Units.inchesToMeters(9), Units.inchesToMeters(4))), new Rotation2d(Units.degreesToRadians(-30))),
        new Pose2d(TrajectoryConstants.BLUE_CENTER_SCORE, new Rotation2d(Units.degreesToRadians(18)))), reverseConfig);

    Trajectory driveToFourthNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.BLUE_CENTER_SCORE, new Rotation2d(Units.degreesToRadians(18))),
        new Pose2d(TrajectoryConstants.NOTE6_BLUE.plus(new Translation2d(Units.inchesToMeters(12), 0)), new Rotation2d(Units.degreesToRadians(-60)))), forwardConfig);

 Trajectory driveToScoreFourthNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.NOTE6_BLUE.plus(new Translation2d(Units.inchesToMeters(12), 0)), new Rotation2d(Units.degreesToRadians(-60))),
        new Pose2d(TrajectoryConstants.BLUE_CENTER_SCORE, new Rotation2d(Units.degreesToRadians(18)))), reverseConfig);

     //Simulation
    //  field = new Field2d();

    //  if (RobotBase.isSimulation()) {
    //     SmartDashboard.putData(field);

    //     field.setRobotPose(driveToSecondNoteTrajectory.getInitialPose());
      
    //     field.getObject("Drive to second Trajectory").setTrajectory(driveToSecondNoteTrajectory);
    //     field.getObject("Drive to score second Trajectory").setTrajectory(driveToScoreSecondNoteTrajectory);
    //     field.getObject("Drive to third Trajectory").setTrajectory(driveToThirdNoteTrajectory);
    //     field.getObject("Drive to score third Trajectory").setTrajectory(driveToScoreThirdNoteTrajectory);
    //     field.getObject("Drive to pick up 4 Trajectory").setTrajectory(driveToFourthNoteTrajectory);
    //     field.getObject("Drive to score 4 Trajectory").setTrajectory(driveToScoreFourthNoteTrajectory);
    //   }

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

    // System.out.println("TIME: " + (driveToSecondNoteTrajectory.getTotalTimeSeconds() + driveToScoreSecondNoteTrajectory.getTotalTimeSeconds() + driveToThirdNoteTrajectory.getTotalTimeSeconds() + driveToScoreThirdNoteTrajectory.getTotalTimeSeconds() + driveToFourthNoteTrajectory.getTotalTimeSeconds() + driveToScoreFourthNoteTrajectory.getTotalTimeSeconds()));


    SwerveControllerCommand driveToSecondNoteCommand = new SwerveControllerCommand(
        driveToSecondNoteTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.kinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    SwerveControllerCommand driveToScoreSecondNoteCommand = new SwerveControllerCommand(
        driveToScoreSecondNoteTrajectory,
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

    SwerveControllerCommand driveToScoreThirdNoteCommand = new SwerveControllerCommand(
        driveToScoreThirdNoteTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.kinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    SwerveControllerCommand driveToFourthNoteCommand = new SwerveControllerCommand(
        driveToFourthNoteTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.kinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    SwerveControllerCommand driveToScoreFourthNoteCommand = new SwerveControllerCommand(
        driveToScoreFourthNoteTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.kinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> driveSubsystem.seedFieldRelative(driveToSecondNoteTrajectory.getInitialPose())), 
        // shoot preload
        new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> shooterAngleSubsystem.getTargetAngle())),
        new ShooterRevUpCommand(shooterSubsystem, 5400),
        new WaitCommand(0.1),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED)),
        new WaitCommand(0.1),
        new InstantCommand(() -> transitionSubsystem.stopTransition()),
        new InstantCommand(() -> shooterSubsystem.stopShooterMotor()),
        // go to and shoot first note (nvm)
        // go to and shoot second note
        new ParallelDeadlineGroup( 
          new SequentialCommandGroup(driveToSecondNoteCommand, driveToScoreSecondNoteCommand), 
          new SequentialCommandGroup(new WaitCommand(AutoConstants.AUTO_INTAKE_DELAY), new IntakeTransitionCommand(transitionSubsystem, intakeSubsystem), new ShooterRevUpCommand(shooterSubsystem, 5400))), 
        new ShooterRevUpCommand(shooterSubsystem, 5400),
        new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> shooterAngleSubsystem.getTargetAngle())),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED)),
        new WaitCommand(0.2),
        new InstantCommand(() -> transitionSubsystem.stopTransition()),
        new InstantCommand(() -> shooterSubsystem.stopShooterMotor()),
        // go to and shoot third note
        new ParallelDeadlineGroup( 
          new SequentialCommandGroup(driveToThirdNoteCommand, driveToScoreThirdNoteCommand), 
          new SequentialCommandGroup(new WaitCommand(AutoConstants.AUTO_INTAKE_DELAY), new IntakeTransitionCommand(transitionSubsystem, intakeSubsystem), new ShooterRevUpCommand(shooterSubsystem, 5400))), 
        new ShooterRevUpCommand(shooterSubsystem, 5400),
        new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> shooterAngleSubsystem.getTargetAngle())),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED)),
        new WaitCommand(0.2),
        new InstantCommand(() -> transitionSubsystem.stopTransition()),
        new InstantCommand(() -> shooterSubsystem.stopShooterMotor()),
        // fourth note
        new ParallelDeadlineGroup( 
          new SequentialCommandGroup(driveToFourthNoteCommand, driveToScoreFourthNoteCommand), 
          new SequentialCommandGroup(new WaitCommand(AutoConstants.AUTO_INTAKE_DELAY), new IntakeTransitionCommand(transitionSubsystem, intakeSubsystem), new ShooterRevUpCommand(shooterSubsystem, 5400))), 
        new ShooterRevUpCommand(shooterSubsystem, 5400),
        new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> shooterAngleSubsystem.getTargetAngle())),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED)),
        new WaitCommand(0.2),
        new InstantCommand(() -> transitionSubsystem.stopTransition()),
        new InstantCommand(() -> shooterSubsystem.stopShooterMotor())
    );
  }
}


