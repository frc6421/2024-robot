// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransitionArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;
import frc.robot.subsystems.TransitionArmSubsystem.TransitionArmConstants;
import frc.robot.subsystems.TransitionSubsystem.TransitionConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.commands.CenterNoteCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final TransitionArmSubsystem armSubsystem;

  // Controllers \\
  private final CommandXboxController driverController; 

  private static final int driverControllerPort = 0;

  // Subsystems \\
  private final DriveSubsystem driveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final TransitionSubsystem transitionSubsystem;

  // Commands \\
  private final DriveCommand driveCommand;
  private final CenterNoteCommand centerNoteCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    driverController = new CommandXboxController(driverControllerPort);

    driveSubsystem = new DriveSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    transitionSubsystem = new TransitionSubsystem();
    armSubsystem = new TransitionArmSubsystem();

    driveCommand = new DriveCommand(driveSubsystem, driverController);
    centerNoteCommand = new CenterNoteCommand(transitionSubsystem);

    driveSubsystem.setDefaultCommand(driveCommand);
    
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
     driverController.leftBumper().whileTrue(new RunCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_IN_SPEED), intakeSubsystem));
     driverController.leftBumper().whileTrue(centerNoteCommand);
     driverController.leftBumper().onFalse(new InstantCommand(() -> intakeSubsystem.stopIntake()));

     driverController.rightBumper().whileTrue(new RunCommand(() -> intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_OUT_SPEED), intakeSubsystem));
     driverController.rightBumper().whileTrue(new InstantCommand(() -> transitionSubsystem.setTransitionMotorOutput(TransitionConstants.TRANSITION_REVERSE_SPEED)));
     driverController.rightBumper().onFalse(new InstantCommand(() -> intakeSubsystem.stopIntake()));
     driverController.rightBumper().onFalse(new InstantCommand(() -> transitionSubsystem.setTransitionMotorOutput(0)));

    driverController.a().onTrue(new InstantCommand(() -> armSubsystem.setArmMotorPosition(90)));

    driverController.x().onTrue(new InstantCommand(() -> transitionSubsystem.setTransitionMotorOutput(TransitionConstants.TRANSITION_FORWARD_SPEED))
      .andThen(new WaitCommand(0.5))
      .andThen(new InstantCommand(() -> transitionSubsystem.setTransitionMotorOutput(0))
      .andThen(new InstantCommand(() -> armSubsystem.setArmMotorPosition(0)))
      .andThen(new InstantCommand(() -> armSubsystem.setArmMotorPosition(TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT)))));
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
