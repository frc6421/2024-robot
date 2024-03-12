// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.subsystems.TransitionSubsystem.TransitionConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedSixPieceCommand extends SequentialCommandGroup {
  private DriveSubsystem driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private TransitionSubsystem transitionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private ShooterAngleSubsystem shooterAngleSubsystem;

   private Field2d field;

  /** Creates a new BlueTwoPieceCommand. */
  public RedSixPieceCommand(DriveSubsystem drive, IntakeSubsystem intake, TransitionSubsystem transition, ShooterSubsystem shooter, ShooterAngleSubsystem shooterAngle) {

    driveSubsystem = drive;
    intakeSubsystem = intake;
    transitionSubsystem = transition;
    shooterSubsystem = shooter;
    shooterAngleSubsystem = shooterAngle;
    addRequirements(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem);

    TrajectoryConfig forwardConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND + 0.3,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(driveSubsystem.kinematics);
    
    TrajectoryConfig reverseConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND + 0.3,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(driveSubsystem.kinematics)
        .setReversed(true);

    // robot leaves start zone and moves to pick up note at podium
    Trajectory driveToFirstNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.FRONT_CENTER_RED_SUBWOOFER, new Rotation2d(Units.degreesToRadians(180))),
        new Pose2d(TrajectoryConstants.NOTE9.minus(new Translation2d(Units.inchesToMeters(7), 0)), new Rotation2d(Units.degreesToRadians(180)))), forwardConfig);

    Trajectory driveToScoreFirstNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.NOTE9.minus(new Translation2d(Units.inchesToMeters(7), 0)), new Rotation2d(Units.degreesToRadians(180))),
        new Pose2d(TrajectoryConstants.FRONT_CENTER_RED_SUBWOOFER.minus(new Translation2d(Units.inchesToMeters(12), 0)), new Rotation2d(Units.degreesToRadians(180)))), reverseConfig);

    Trajectory driveToSecondNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.FRONT_CENTER_RED_SUBWOOFER.minus(new Translation2d(Units.inchesToMeters(12), 0)), new Rotation2d(Units.degreesToRadians(180))),
        new Pose2d(TrajectoryConstants.NOTE10.minus(new Translation2d(Units.inchesToMeters(9), 0)), new Rotation2d(Units.degreesToRadians(180)))), forwardConfig);

    Trajectory driveToPrepThirdNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.NOTE10.minus(new Translation2d(Units.inchesToMeters(9), Units.inchesToMeters(0))), new Rotation2d(Units.degreesToRadians(180))),
        new Pose2d(TrajectoryConstants.NOTE11.plus(new Translation2d(Units.inchesToMeters(24), Units.inchesToMeters(-12))), new Rotation2d(Units.degreesToRadians(180)))), reverseConfig);

    Trajectory driveToThirdNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.NOTE11.plus(new Translation2d(Units.inchesToMeters(24), Units.inchesToMeters(-12))), new Rotation2d(Units.degreesToRadians(180))),
        new Pose2d(TrajectoryConstants.NOTE11.minus(new Translation2d(Units.inchesToMeters(12), 0)), new Rotation2d(Units.degreesToRadians(143.84)))), forwardConfig);

     Trajectory driveToFourthNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.NOTE11.minus(new Translation2d(Units.inchesToMeters(12), 0)), new Rotation2d(Units.degreesToRadians(143.84))),
        new Pose2d(TrajectoryConstants.RED_DONT_HIT_WALL.minus(new Translation2d(Units.inchesToMeters(55), 0)), new Rotation2d(Units.degreesToRadians(180))),
        new Pose2d(TrajectoryConstants.NOTE8_RED, new Rotation2d(Units.degreesToRadians(180)))), forwardConfig);

    Trajectory driveToScoreFourthNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.NOTE8_RED, new Rotation2d(Units.degreesToRadians(180))),
        new Pose2d(TrajectoryConstants.RED_CENTER_SCORE, new Rotation2d(Units.degreesToRadians(170)))), reverseConfig);

     Trajectory driveToFifthNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.RED_CENTER_SCORE, new Rotation2d(Units.degreesToRadians(170))),
        new Pose2d(TrajectoryConstants.NOTE7_RED, new Rotation2d(Units.degreesToRadians(210)))), forwardConfig);

    Trajectory driveToScoreFifthNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.NOTE7_RED, new Rotation2d(Units.degreesToRadians(210))),
        new Pose2d(TrajectoryConstants.RED_CENTER_SCORE, new Rotation2d(Units.degreesToRadians(164.03)))), reverseConfig);

    // System.out.println("TIME: " + (driveToFirstNoteTrajectory.getTotalTimeSeconds() + driveToScoreFirstNoteTrajectory.getTotalTimeSeconds() + driveToSecondNoteTrajectory.getTotalTimeSeconds() + driveToThirdNoteTrajectory.getTotalTimeSeconds() + driveToFourthNoteTrajectory.getTotalTimeSeconds() + driveToScoreFourthNoteTrajectory.getTotalTimeSeconds() + driveToFifthNoteTrajectory.getTotalTimeSeconds() + driveToScoreFifthNoteTrajectory.getTotalTimeSeconds()));

    // Simulation
    //  field = new Field2d();

    //  if (RobotBase.isSimulation()) {
    //     SmartDashboard.putData(field);

    //     field.setRobotPose(driveToFirstNoteTrajectory.getInitialPose());
      
    //     field.getObject("Drive to first Trajectory").setTrajectory(driveToFirstNoteTrajectory);
    //     field.getObject("Drive to score 1 Trajectory").setTrajectory(driveToScoreFirstNoteTrajectory);
    //     field.getObject("Drive to second Trajectory").setTrajectory(driveToSecondNoteTrajectory);
    //     field.getObject("Drive to prep third Trajectory").setTrajectory(driveToPrepThirdNoteTrajectory);
    //     field.getObject("Drive to third Trajectory").setTrajectory(driveToThirdNoteTrajectory);
    //     field.getObject("Drive to 4 Trajectory").setTrajectory(driveToFourthNoteTrajectory);
    //     field.getObject("Drive to score 4 Trajectory").setTrajectory(driveToScoreFourthNoteTrajectory);
    //     field.getObject("Drive to 5 Trajectory").setTrajectory(driveToFifthNoteTrajectory);
    //     field.getObject("Drive to score 5 Trajectory").setTrajectory(driveToScoreFifthNoteTrajectory);
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

    SwerveControllerCommand driveToFirstNoteCommand = new SwerveControllerCommand(
        driveToFirstNoteTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.kinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    SwerveControllerCommand driveToScoreFirstNoteCommand = new SwerveControllerCommand(
        driveToScoreFirstNoteTrajectory,
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

    SwerveControllerCommand driveToPrepThirdNoteCommand = new SwerveControllerCommand(
        driveToPrepThirdNoteTrajectory,
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

    SwerveControllerCommand driveToFifthNoteCommand = new SwerveControllerCommand(
        driveToFifthNoteTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.kinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    SwerveControllerCommand driveToScoreFifthNoteCommand = new SwerveControllerCommand(
        driveToScoreFifthNoteTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.kinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> driveSubsystem.seedFieldRelative(driveToFirstNoteTrajectory.getInitialPose())), 
        // shoot preload
        new InstantCommand(() -> shooterAngleSubsystem.setAngle(50)),
        new ShooterRevUpCommand(shooterSubsystem, 4500),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED)),
        new WaitCommand(0.15),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(0)),
        new InstantCommand(() -> shooterSubsystem.stopShooterMotor()),
        // go to and shoot first note
        new ParallelDeadlineGroup( 
          new SequentialCommandGroup(driveToFirstNoteCommand, driveToScoreFirstNoteCommand), 
          new IntakeTransitionCommand(transitionSubsystem, intakeSubsystem)), 
        new InstantCommand(() -> shooterAngleSubsystem.setAngle(33)),
        new ShooterRevUpCommand(shooterSubsystem, 4000),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED)),
        new WaitCommand(0.15),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(0)),
        new InstantCommand(() -> shooterSubsystem.stopShooterMotor()),
        // go to and shoot second note
        new ParallelDeadlineGroup( 
          new SequentialCommandGroup(driveToSecondNoteCommand), 
          new IntakeTransitionCommand(transitionSubsystem, intakeSubsystem)), 
        new InstantCommand(() -> shooterAngleSubsystem.setAngle(33)),
        new ShooterRevUpCommand(shooterSubsystem, 4000),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED)),
        new WaitCommand(0.15),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(0)),
        new InstantCommand(() -> shooterSubsystem.stopShooterMotor()),
        // go to and shoot third note
        new ParallelDeadlineGroup( 
          new SequentialCommandGroup(driveToPrepThirdNoteCommand, driveToThirdNoteCommand), 
          new IntakeTransitionCommand(transitionSubsystem, intakeSubsystem)), 
        new InstantCommand(() -> shooterAngleSubsystem.setAngle(33)),
        new ShooterRevUpCommand(shooterSubsystem, 4000),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED)),
        new WaitCommand(0.15),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(0)),
        new InstantCommand(() -> shooterSubsystem.stopShooterMotor()),
        // fourth note slay
        new ParallelDeadlineGroup( 
          new SequentialCommandGroup(driveToFourthNoteCommand, driveToScoreFourthNoteCommand), 
          new IntakeTransitionCommand(transitionSubsystem, intakeSubsystem)), 
        new InstantCommand(() -> shooterAngleSubsystem.setAngle(27)),
        new ShooterRevUpCommand(shooterSubsystem, 4500),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED)),
        new WaitCommand(0.15),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(0)),
        new InstantCommand(() -> shooterSubsystem.stopShooterMotor()),
        // fifth note maybe
        new ParallelDeadlineGroup( 
          new SequentialCommandGroup(driveToFifthNoteCommand, driveToScoreFifthNoteCommand), 
          new IntakeTransitionCommand(transitionSubsystem, intakeSubsystem)), 
        new InstantCommand(() -> shooterAngleSubsystem.setAngle(27)),
        new ShooterRevUpCommand(shooterSubsystem, 4500),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED)),
        new WaitCommand(0.15),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(0)),
        new InstantCommand(() -> shooterSubsystem.stopShooterMotor())
    );
  }
}
