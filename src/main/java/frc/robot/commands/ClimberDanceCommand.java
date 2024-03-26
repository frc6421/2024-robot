// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberStates;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.TransitionArmSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberConstants;
import frc.robot.subsystems.TransitionArmSubsystem.TransitionArmConstants;
import frc.robot.subsystems.TransitionSubsystem.TransitionConstants;

public class ClimberDanceCommand extends Command {
  /** Creates a new ClimberDance. */
  private final ClimberSubsystem climberSubsystem;
  private final TransitionArmSubsystem transitionArmSubsystem;
  private final TransitionSubsystem transitionSubsystem;
  //private final DriveSubsystem driveSubsystem;

  private final static double TRANSITION_ARM_MID_ANGLE = 25.0;
  private final static double TRANSITION_ARM_HIGH_ANGLE = 105;
  private final static double TRANSITION_ARM_TRAP_ANGLE = TransitionArmConstants.ARM_FORWARD_SOFT_LIMIT;

  // TODO what are these numbers
  private final static double CLIMBER_MID_ROTATIONS = -3620;
  //private final static double CLIMBER_HIGH_ROTATIONS = 0;
  private final static double CLIMBER_IN_VOLTAGE = -12;

  private boolean exitCommand = false;

  public ClimberDanceCommand(ClimberSubsystem climberSubsystem, TransitionArmSubsystem transitionArmSubsystem, TransitionSubsystem transitionSubsystem) {
    addRequirements(climberSubsystem, transitionArmSubsystem, transitionSubsystem);

    this.climberSubsystem = climberSubsystem;
    this.transitionArmSubsystem = transitionArmSubsystem;
    this.transitionSubsystem = transitionSubsystem;

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
    exitCommand = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(RobotContainer.climberState) {
      case PREPARE_CLIMB:
        //new ArmCommand(transitionArmSubsystem, TRANSITION_ARM_MID_ANGLE, 0);
        climberSubsystem.setClimberMotorPosition(CLIMBER_MID_ROTATIONS, 0);
        // TODO DRIVE
        // Wheels to coast, Point wheels toward the stage, Drive back 
        RobotContainer.climberState = ClimberStates.CLIMB;
        exitCommand = true;
      break;
      case CLIMB:
        //new ArmCommand(transitionArmSubsystem, TRANSITION_ARM_HIGH_ANGLE, 1);
        //climberSubsystem.setClimberMotorPosition(CLIMBER_HIGH_ROTATIONS, 1);
        //RobotContainer.climberState = ClimberStates.PREPARE_TRAP;

        climberSubsystem.setClimberVoltage(CLIMBER_IN_VOLTAGE);
      break;
      // case ARMS_HIGH:
      //   //new ArmCommand(transitionArmSubsystem, TRANSITION_ARM_HIGH_ANGLE, 0);
      //   climberSubsystem.setClimberMotorPosition(CLIMBER_HIGH_ROTATIONS, 0);
      //   RobotContainer.climberState = ClimberStates.CLIMB;

      //   exitCommand = true;
      // break;
      // case PREPARE_TRAP:
      //   transitionArmSubsystem.setArmMotorPosition(TRANSITION_ARM_TRAP_ANGLE, 1);
      //   RobotContainer.climberState = ClimberStates.SCORE_TRAP;

      //   exitCommand = true;
      // break;
      // case SCORE_TRAP:

      //   new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED))
      //     .andThen(new WaitCommand(0.4))
      //     .andThen(new InstantCommand(() -> transitionSubsystem.stopTransition()));

      //   // TODO Reverse climb?

      //   exitCommand = true;
      // break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if(!exitCommand) {
      climberSubsystem.setClimberVoltage(0);
    }
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (climberSubsystem.getClimberLeftMotorPosition() <= ClimberConstants.CLIMBER_REVERSE_SOFT_LIMIT_ROTATIONS && climberSubsystem.getClimberRightMotorPosition() <= ClimberConstants.CLIMBER_REVERSE_SOFT_LIMIT_ROTATIONS) || exitCommand;
  }
}
