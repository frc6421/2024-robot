// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

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
import frc.robot.Constants.RobotStates;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.IntakeTransitionCommand;
import frc.robot.commands.ShooterRevUpCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionArmSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem.AngleConstants;
import frc.robot.subsystems.TransitionArmSubsystem.TransitionArmConstants;
import frc.robot.subsystems.TransitionSubsystem.TransitionConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedAmpThreePieceCommand extends SequentialCommandGroup {
  private DriveSubsystem driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private TransitionSubsystem transitionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private ShooterAngleSubsystem shooterAngleSubsystem;
  private TransitionArmSubsystem armSubsystem;

  private Field2d field;
  /** Creates a new BlueTwoPieceCommand. */
  public RedAmpThreePieceCommand(DriveSubsystem drive, IntakeSubsystem intake, TransitionSubsystem transition, ShooterSubsystem shooter, ShooterAngleSubsystem shooterAngle, TransitionArmSubsystem armSubsystem) {

    driveSubsystem = drive;
    intakeSubsystem = intake;
    transitionSubsystem = transition;
    shooterSubsystem = shooter;
    shooterAngleSubsystem = shooterAngle;
    addRequirements(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem, armSubsystem);

    TrajectoryConfig forwardConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED - 1)
        .setKinematics(driveSubsystem.kinematics);
    
    TrajectoryConfig reverseConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED - 1)
        .setKinematics(driveSubsystem.kinematics)
        .setReversed(true);

    // robot leaves start zone and moves to pick up note at podium
    Trajectory driveToAmpTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.RED_AMP_START, new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(TrajectoryConstants.RED_AMP, new Rotation2d(Units.degreesToRadians(270)))), reverseConfig);

    Trajectory driveToFirstNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.RED_AMP, new Rotation2d(Units.degreesToRadians(270))),
        new Pose2d(TrajectoryConstants.NOTE11.minus(new Translation2d(Units.inchesToMeters(12), Units.inchesToMeters(0))), new Rotation2d(Units.degreesToRadians(180)))), forwardConfig);

    Trajectory driveToScoreNote1Trajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.NOTE11.minus(new Translation2d(Units.inchesToMeters(12), Units.inchesToMeters(0))), new Rotation2d(Units.degreesToRadians(180))),
        new Pose2d(TrajectoryConstants.RED_AMP, new Rotation2d(Units.degreesToRadians(270)))), reverseConfig);

    Trajectory driveToSecondNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.RED_AMP, new Rotation2d(Units.degreesToRadians(270))),
        new Pose2d(TrajectoryConstants.NOTE11, new Rotation2d(Units.degreesToRadians(180))),
        new Pose2d(TrajectoryConstants.NOTE8_RED.plus(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0))), new Rotation2d(Units.degreesToRadians(180)))), forwardConfig);

    Trajectory driveToScoreNote2Trajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.NOTE8_RED.plus(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0))), new Rotation2d(Units.degreesToRadians(180))),
        new Pose2d(TrajectoryConstants.NOTE11, new Rotation2d(Units.degreesToRadians(180))),
        new Pose2d(TrajectoryConstants.RED_AMP, new Rotation2d(Units.degreesToRadians(270)))), reverseConfig);



    // Simulation
    // field = new Field2d();

    // if (RobotBase.isSimulation()) {
    //    SmartDashboard.putData(field);

    //    field.setRobotPose(driveToAmpTrajectory.getInitialPose());
      
    //    field.getObject("Drive to Amp Trajectory").setTrajectory(driveToAmpTrajectory);
    //   field.getObject("Drive to Note1 Trajectory").setTrajectory(driveToFirstNoteTrajectory);
    //   field.getObject("Drive to Score Note1 Trajectory").setTrajectory(driveToScoreNote1Trajectory);
    //   field.getObject("Drive to Note2 Trajectory").setTrajectory(driveToSecondNoteTrajectory);
    //   field.getObject("Drive to Score Note2 Trajectory").setTrajectory(driveToScoreNote2Trajectory);

    //  }

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

    SwerveControllerCommand driveToAmpCommand = new SwerveControllerCommand(
        driveToAmpTrajectory,
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

    SwerveControllerCommand driveToScoreNote1Command = new SwerveControllerCommand(
        driveToScoreNote1Trajectory,
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

    SwerveControllerCommand driveToScoreNote2Command = new SwerveControllerCommand(
        driveToScoreNote2Trajectory,
        driveSubsystem::getPose2d,
        driveSubsystem.kinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> driveSubsystem.seedFieldRelative(driveToAmpTrajectory.getInitialPose())), 
      new InstantCommand(() -> driveSubsystem.setGyroAngleCommand(driveToAmpTrajectory.getInitialPose().getRotation().getDegrees())),
      driveToAmpCommand,
      // amp preload
      new ParallelCommandGroup(new InstantCommand(() -> shooterSubsystem.stopShooterMotor()), 
          new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES)))
        .andThen(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_AMP_POSITION, 0))
        .andThen(new WaitCommand(0.3))
        .andThen(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.AMP_TRANSITION_SPEED)))
        .andThen(new WaitCommand(0.4))
        .andThen(new InstantCommand(() -> transitionSubsystem.stopTransition()))
        .andThen(new InstantCommand(() -> shooterSubsystem.stopShooterMotor()))
        .andThen(new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES)))
        .andThen(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT, 0)),

     // pickup second note
     new ParallelDeadlineGroup( 
          new SequentialCommandGroup(driveToFirstNoteCommand, driveToScoreNote1Command), 
          new SequentialCommandGroup(new WaitCommand(AutoConstants.AUTO_INTAKE_DELAY), new IntakeTransitionCommand(transitionSubsystem, intakeSubsystem))),    

    // amp
    new ParallelCommandGroup(new InstantCommand(() -> shooterSubsystem.stopShooterMotor()), 
          new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES)))
        .andThen(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_AMP_POSITION, 0))
        .andThen(new WaitCommand(0.3))
        .andThen(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.AMP_TRANSITION_SPEED)))
        .andThen(new WaitCommand(0.4))
        .andThen(new InstantCommand(() -> transitionSubsystem.stopTransition()))
        .andThen(new InstantCommand(() -> shooterSubsystem.stopShooterMotor()))
        .andThen(new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES)))
        .andThen(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT, 0)),

      // pickup third note
    new ParallelDeadlineGroup( 
          new SequentialCommandGroup(driveToSecondNoteCommand, driveToScoreNote2Command), 
          new SequentialCommandGroup(new WaitCommand(AutoConstants.AUTO_INTAKE_DELAY), new IntakeTransitionCommand(transitionSubsystem, intakeSubsystem))), 
    
     // amp
    new ParallelCommandGroup(new InstantCommand(() -> shooterSubsystem.stopShooterMotor()), 
          new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES)))
        .andThen(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_AMP_POSITION, 0))
        .andThen(new WaitCommand(0.3))
        .andThen(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.AMP_TRANSITION_SPEED)))
        .andThen(new WaitCommand(0.4))
        .andThen(new InstantCommand(() -> transitionSubsystem.stopTransition()))
        .andThen(new InstantCommand(() -> shooterSubsystem.stopShooterMotor()))
        .andThen(new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES)))
        .andThen(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT, 0)),

      new InstantCommand(() -> driveSubsystem.setControl(new SwerveRequest.ApplyChassisSpeeds()))
    );
  }
}
