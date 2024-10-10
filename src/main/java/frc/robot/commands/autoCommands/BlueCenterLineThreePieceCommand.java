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
public class BlueCenterLineThreePieceCommand extends SequentialCommandGroup {
  private DriveSubsystem driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private TransitionSubsystem transitionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private ShooterAngleSubsystem shooterAngleSubsystem;

  private Field2d field;

  /** Creates a new BlueTwoPieceCommand. */
  public BlueCenterLineThreePieceCommand(DriveSubsystem drive, IntakeSubsystem intake, TransitionSubsystem transition, ShooterSubsystem shooter, ShooterAngleSubsystem shooterAngle) {

    driveSubsystem = drive;
    intakeSubsystem = intake;
    transitionSubsystem = transition;
    shooterSubsystem = shooter;
    shooterAngleSubsystem = shooterAngle;
    addRequirements(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem);

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
    Trajectory scorePreloadTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.BLUE_CENTER_LINE_STARTING_POSITION, new Rotation2d(Units.degreesToRadians(180))), 
        new Pose2d(TrajectoryConstants.BLUE_CENTER_OF_STAGE, new Rotation2d(Units.degreesToRadians(-90))),
        new Pose2d(TrajectoryConstants.BLUE_CENTER_LINE_SHOOTING_POSITION, new Rotation2d(Units.degreesToRadians(-9)))), reverseConfig);

    Trajectory driveToFirstNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.BLUE_CENTER_LINE_SHOOTING_POSITION, new Rotation2d(Units.degreesToRadians(-9))),
        new Pose2d(TrajectoryConstants.BLUE_EDGE_OF_STAGE, new Rotation2d(Units.degreesToRadians(-25))),
        new Pose2d(TrajectoryConstants.NOTE5_BLUE, new Rotation2d(0))), forwardConfig);

    Trajectory driveBackToScoreOneTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.NOTE5_BLUE, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.BLUE_EDGE_OF_STAGE, new Rotation2d(Units.degreesToRadians(-25))),
        new Pose2d(TrajectoryConstants.BLUE_CENTER_LINE_SHOOTING_POSITION, new Rotation2d(Units.degreesToRadians(-9)))), reverseConfig);

    Trajectory driveToSecondNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.BLUE_CENTER_LINE_SHOOTING_POSITION, new Rotation2d(Units.degreesToRadians(-9))),
        new Pose2d(TrajectoryConstants.BLUE_EDGE_OF_STAGE, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.NOTE6_BLUE, new Rotation2d(0))), forwardConfig);

    Trajectory driveBackToScoreTwoTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.NOTE6_BLUE, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.BLUE_EDGE_OF_STAGE, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.BLUE_CENTER_LINE_SHOOTING_POSITION, new Rotation2d(Units.degreesToRadians(-9)))), reverseConfig);

    


    // Simulation
    //  field = new Field2d();

    //  if (RobotBase.isSimulation()) {
    //     SmartDashboard.putData(field);

    //     field.setRobotPose(scorePreloadTrajectory.getInitialPose());

    //     field.getObject("1 Preload").setTrajectory(scorePreloadTrajectory);


    //      field.getObject("Drive to first Trajectory").setTrajectory(driveToFirstNoteTrajectory);
    //      field.getObject("Drive to score one Trajectory").setTrajectory(driveBackToScoreOneTrajectory);
    //      field.getObject("Drive to second Trajectory").setTrajectory(driveToSecondNoteTrajectory);
    //      field.getObject("Drive to score two Trajectory").setTrajectory(driveBackToScoreTwoTrajectory);
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

    SwerveControllerCommand scorePreloadCommand = new SwerveControllerCommand(
        scorePreloadTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.kinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    SwerveControllerCommand driveToFirstNoteCommand = new SwerveControllerCommand(
        driveToFirstNoteTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.kinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    SwerveControllerCommand driveBackToScoreOneCommand = new SwerveControllerCommand(
        driveBackToScoreOneTrajectory,
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

    SwerveControllerCommand driveBackToScoreTwoCommand = new SwerveControllerCommand(
        driveBackToScoreTwoTrajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.kinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> driveSubsystem.seedFieldRelative(scorePreloadTrajectory.getInitialPose())), 
        new ShooterRevUpCommand(shooterSubsystem, 5400),
        scorePreloadCommand, 
        new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> shooterAngleSubsystem.getTargetAngle())),
        new WaitCommand(0.3),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED)),
        new WaitCommand(0.1),
        new InstantCommand(() -> transitionSubsystem.stopTransition()),
        new InstantCommand(() -> shooterSubsystem.stopShooterMotor()),
        // score pre-loaded piece 
        new ParallelDeadlineGroup( 
          new SequentialCommandGroup(driveToFirstNoteCommand, driveBackToScoreOneCommand), 
          new SequentialCommandGroup(new WaitCommand(AutoConstants.AUTO_INTAKE_DELAY), new IntakeTransitionCommand(transitionSubsystem, intakeSubsystem), new ShooterRevUpCommand(shooterSubsystem, 5400))), 
        new ShooterRevUpCommand(shooterSubsystem, 5400),
        new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> shooterAngleSubsystem.getTargetAngle())),
        new WaitCommand(0.1),
        // new ShooterRevUpCommand(shooterSubsystem, 5400),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED)),
        new WaitCommand(0.1),
        new InstantCommand(() -> transitionSubsystem.stopTransition()),
        new InstantCommand(() -> shooterSubsystem.stopShooterMotor()),
        // stop intake and score second piece
        new ParallelDeadlineGroup( 
          new SequentialCommandGroup(driveToSecondNoteCommand, driveBackToScoreTwoCommand), 
          new SequentialCommandGroup(new WaitCommand(AutoConstants.AUTO_INTAKE_DELAY), new IntakeTransitionCommand(transitionSubsystem, intakeSubsystem), new ShooterRevUpCommand(shooterSubsystem, 5400))), 
        new ShooterRevUpCommand(shooterSubsystem, 5400),
        new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> shooterAngleSubsystem.getTargetAngle())),
        new WaitCommand(0.1),
        // new ShooterRevUpCommand(shooterSubsystem, 5400),
        new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED)),
        new WaitCommand(0.1),
        new InstantCommand(() -> transitionSubsystem.stopTransition()),
        new InstantCommand(() -> shooterSubsystem.stopShooterMotor())
    );
  }
}
