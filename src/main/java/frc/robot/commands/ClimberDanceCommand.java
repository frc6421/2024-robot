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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TransitionArmSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.subsystems.TransitionSubsystem.TransitionConstants;

public class ClimberDanceCommand extends Command {
  /** Creates a new ClimberDance. */
  private final ClimberSubsystem climberSubsystem;
  private final TransitionArmSubsystem transitionArmSubsystem;
  private final TransitionSubsystem transitionSubsystem;
  //private final DriveSubsystem driveSubsystem;

  private static enum ClimberStates {
    NOT_CLIMBING, // State when the robot is not climbing/trapping (do we need this state?)
    PREPARE_CLIMB, // Raise both arms up to middle position, drive backwards.
    ARMS_HIGH, // Raise both arms to the climbing position
    CLIMB, // Lower climber arms
    PREPARE_TRAP, // Raise transition arm to trap degree
    SCORE_TRAP // Spit out note
  }

  private final static double TRANSITION_ARM_MID_ANGLE = 25.0;
  private final static double TRANSITION_ARM_HIGH_ANGLE = 90.0;
  private final static double TRANSITION_ARM_TRAP_ANGLE = 110.0;

  private final static double CLIMBER_LOW_ROTATIONS = 0.0;
  private final static double CLIMBER_MID_ROTATIONS = 0.0;
  private final static double CLIMBER_HIGH_ROTATIONS = 0.0;

  private int counter;

  private ClimberStates climberStates;
  public ClimberDanceCommand(ClimberSubsystem climberSubsystem, TransitionArmSubsystem transitionArmSubsystem, TransitionSubsystem transitionSubsystem) {
    addRequirements(climberSubsystem, transitionArmSubsystem, transitionSubsystem);

    this.climberSubsystem = climberSubsystem;
    this.transitionArmSubsystem = transitionArmSubsystem;
    this.transitionSubsystem = transitionSubsystem;
    climberStates = ClimberStates.PREPARE_CLIMB;

    // TrajectoryConfig config = new TrajectoryConfig(
    //       AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND - 2,
    //       AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED - 2)
    //       .setKinematics(driveSubsystem.kinematics)
    //       .setReversed(true);

    // //TODO change endpoint to actually correct number
    // Trajectory driveUnderStageTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     new Pose2d(Units.feetToMeters(-2), 0, new Rotation2d(0))), config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.THETA_P, AutoConstants.THETA_I, AutoConstants.THETA_D,
    //     new TrapezoidProfile.Constraints(AutoConstants.AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
    //         AutoConstants.AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC));
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
    //     // Position controllers
    //     new PIDController(AutoConstants.X_DRIVE_P, AutoConstants.X_DRIVE_I, AutoConstants.X_DRIVE_D),
    //     new PIDController(AutoConstants.Y_DRIVE_P, AutoConstants.Y_DRIVE_I, AutoConstants.Y_DRIVE_D),
    //     thetaController);

    // SwerveControllerCommand driveUnderStageCommand = new SwerveControllerCommand(
    //     driveUnderStageTrajectory,
    //     driveSubsystem::getPose2d,
    //     driveSubsystem.kinematics,
    //     holonomicDriveController,
    //     driveSubsystem::autoSetModuleStates,
    //     driveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    counter++;

    switch(climberStates) {
      case PREPARE_CLIMB:
        transitionArmSubsystem.setArmMotorPosition(TRANSITION_ARM_MID_ANGLE);
        climberSubsystem.setClimberMotorPosition(CLIMBER_MID_ROTATIONS);
        // TODO DRIVE
        // Wheels to coast, Point wheels toward the stage, Drive back 
        climberStates = ClimberStates.ARMS_HIGH;
      break;
      case ARMS_HIGH:
        transitionArmSubsystem.setArmMotorPosition(TRANSITION_ARM_HIGH_ANGLE);
        climberSubsystem.setClimberMotorPosition(CLIMBER_HIGH_ROTATIONS);
        climberStates = ClimberStates.CLIMB;
      break;
      case CLIMB:
        climberSubsystem.setClimberMotorPosition(CLIMBER_LOW_ROTATIONS);
        climberStates = ClimberStates.PREPARE_TRAP;
      break;
      case PREPARE_TRAP:
        transitionArmSubsystem.setArmMotorPosition(TRANSITION_ARM_TRAP_ANGLE);
        climberStates = ClimberStates.SCORE_TRAP;
      break;
      case SCORE_TRAP:
        transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED);
        new WaitCommand(0.4);
        transitionSubsystem.stopTransition();
        // TODO Reverse climb?
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter >= 1;
  }
}
