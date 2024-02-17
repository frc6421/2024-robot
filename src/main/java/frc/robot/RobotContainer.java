// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransitionArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;
import frc.robot.subsystems.ShooterAngleSubsystem.AngleConstants;
import frc.robot.subsystems.TransitionArmSubsystem.TransitionArmConstants;
import frc.robot.subsystems.TransitionSubsystem.TransitionConstants;
import frc.robot.subsystems.CANdleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.commands.IntakeTransitionCommand;
import frc.robot.Constants.RobotStates;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShooterAngleCommand;
import frc.robot.commands.ShooterRevUpCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Controllers \\
  private final CommandXboxController driverController; 

  private static final int driverControllerPort = 0;

  // Subsystems \\
  private final DriveSubsystem driveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final TransitionSubsystem transitionSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ShooterAngleSubsystem shooterAngleSubsystem;
  private final TransitionArmSubsystem armSubsystem;

  // Commands \\
  private final DriveCommand driveCommand;

  private final IntakeTransitionCommand intakeTransitionCommand;

<<<<<<< HEAD
  public static RobotStates state;
=======
  private final ShooterAngleCommand shooterAngleCommand;
  private final ShooterRevUpCommand shooterRevUpCommand;
>>>>>>> ShooterCommands

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    driverController = new CommandXboxController(driverControllerPort);

    driveSubsystem = new DriveSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    transitionSubsystem = new TransitionSubsystem();
    armSubsystem = new TransitionArmSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    shooterAngleSubsystem = new ShooterAngleSubsystem();

    driveCommand = new DriveCommand(driveSubsystem, driverController);
    intakeTransitionCommand = new IntakeTransitionCommand(transitionSubsystem, intakeSubsystem);
    shooterAngleCommand = new ShooterAngleCommand(shooterAngleSubsystem);
    shooterRevUpCommand = new ShooterRevUpCommand(shooterSubsystem);

    driveSubsystem.setDefaultCommand(driveCommand);

    state = RobotStates.DRIVE;
    

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Driver Controller: drive controls (left/right joystick), barf (left trigger - set to run as a button), intake (left bumper), score (right bumper)
    // Operator Controller: change scoring location (4 states), A - amp, X - climb, Y - trap, B - speaker

    // INTAKE STATE \\
    driverController.leftBumper().toggleOnTrue(intakeTransitionCommand);

    // BARF STATE \\
    driverController.rightBumper().onTrue(new InstantCommand(() -> state = RobotStates.BARF));
    driverController.rightBumper().whileTrue(new RunCommand(() -> intakeSubsystem.setIntakeVoltage(IntakeConstants.INTAKE_OUT_SPEED), intakeSubsystem));
    driverController.rightBumper().whileTrue(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(-1.0 * TransitionConstants.TRANSITION_SPEED)));
    driverController.rightBumper().onFalse(new InstantCommand(() -> intakeSubsystem.stopIntake()));
    driverController.rightBumper().onFalse(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(0)));
    driverController.rightBumper().onFalse(new InstantCommand(() -> state = RobotStates.DRIVE));

    // AMP STATE \\ 
    driverController.a().onTrue(new InstantCommand(() -> armSubsystem.setArmMotorPosition(90)));

    driverController.x().onTrue(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED))
      .andThen(new WaitCommand(0.5))
      .andThen(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(0))
      .andThen(new InstantCommand(() -> armSubsystem.setArmMotorPosition(0)))
      .andThen(new InstantCommand(() -> armSubsystem.setArmMotorPosition(TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT)))));
<<<<<<< HEAD

    // SHOOT STATE \\

    // TRAP STATE \\ 

    // CLIMB STATE \\
=======
    
    driverController.y().whileTrue(new ParallelCommandGroup(shooterRevUpCommand, shooterAngleCommand)
      .andThen(new InstantCommand(() -> CANdleSubsystem.setPattern(1, 0, 4)))
      .andThen(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED))));
    driverController.y().onFalse(new InstantCommand(() -> shooterSubsystem.setShooterMotorVelocity(0))
      .andThen(new InstantCommand(() -> shooterAngleSubsystem.setAngle(AngleConstants.MINNIMUM_SOFT_LIMIT_DEGREES)))
      .andThen(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED))));
>>>>>>> ShooterCommands
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
