// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.BlueCenterLineFourPieceCommand;
import frc.robot.commands.BlueCenterLineThreePieceCommand;
import frc.robot.commands.BlueFivePieceCommand;
import frc.robot.commands.BlueFourPieceCommand;
import frc.robot.commands.BlueTwoPieceCommand;
import frc.robot.commands.ClimberDanceCommand;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransitionArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;
import frc.robot.subsystems.LEDSubsystem.LEDConstants.LEDColors;
import frc.robot.subsystems.ShooterAngleSubsystem.AngleConstants;
import frc.robot.subsystems.TransitionArmSubsystem.TransitionArmConstants;
import frc.robot.subsystems.TransitionSubsystem.TransitionConstants;

import java.util.Map;

import com.ctre.phoenix.unmanaged.Unmanaged;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.commands.AmpVisionCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FlipBlueCenterLineFourPieceCommand;
import frc.robot.commands.FlipRedCenterLineFourPieceCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.commands.IntakeTransitionCommand;
import frc.robot.commands.RedCenterLineFourPieceCommand;
import frc.robot.commands.RedCenterLineThreePieceCommand;
import frc.robot.commands.RedFivePieceCommand;
import frc.robot.commands.RedFourPieceCommand;
import frc.robot.commands.RedTwoPieceCommand;
import frc.robot.commands.ShooterRevUpCommand;
import frc.robot.Constants.ClimberStates;
import frc.robot.commands.SpeakerVisionCommand;
import frc.robot.commands.TrapVisionCommand;
import frc.robot.Constants.RobotStates;
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
  private final CommandXboxController operatorController;
  private final CommandXboxController testingcontroller;

  private static final int driverControllerPort = 0;
  private static final int operatorControllerPort = 1;
  private static final int testingcontrollerPort = 2;

  // Subsystems \\
  private final DriveSubsystem driveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final TransitionSubsystem transitionSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ShooterAngleSubsystem shooterAngleSubsystem;
  private final TransitionArmSubsystem armSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final ClimberSubsystem climberSubsystem;

  // Commands \\
  private final DriveCommand driveCommand;
  private final IntakeTransitionCommand intakeTransitionCommand;
  private final AmpVisionCommand ampVisionCommand;
  private final SpeakerVisionCommand speakerVisionCommand;
  private final TrapVisionCommand trapVisionCommand;

  private final BlueTwoPieceCommand blueTwoPiece;
  private final RedTwoPieceCommand redTwoPiece;
  private final BlueFourPieceCommand blueFourPiece;
  private final RedFourPieceCommand redFourPiece;
  private final BlueCenterLineThreePieceCommand blueCenterLineThreePiece;
  private final RedCenterLineThreePieceCommand redCenterLineThreePiece;
  // private final BlueSixPieceCommand blueSixPiece;
  // private final RedSixPieceCommand redSixPiece;
  private final BlueCenterLineFourPieceCommand blueCenterLineFourPiece;
  private final RedCenterLineFourPieceCommand redCenterLineFourPiece;
  private final FlipBlueCenterLineFourPieceCommand flipBlueCenterLineFourPiece;
  private final FlipRedCenterLineFourPieceCommand flipRedCenterLineFourPiece;
  private final BlueFivePieceCommand blueFivePiece;
  private final RedFivePieceCommand redFivePiece;

  
  public static RobotStates robotState;
  public static ClimberStates climberState;

  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Comment this line out to use TunerX, uncomment to improve CAN utilization
    Unmanaged.setPhoenixDiagnosticsStartTime(-1);

    driverController = new CommandXboxController(driverControllerPort);
    operatorController = new CommandXboxController(operatorControllerPort);
    testingcontroller = new CommandXboxController(testingcontrollerPort);

    DriverStation.silenceJoystickConnectionWarning(true);

    driveSubsystem = new DriveSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    transitionSubsystem = new TransitionSubsystem();
    armSubsystem = new TransitionArmSubsystem();
    climberSubsystem = new ClimberSubsystem();
    ledSubsystem = new LEDSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    shooterAngleSubsystem = new ShooterAngleSubsystem();

    driveCommand = new DriveCommand(driveSubsystem, driverController);
    intakeTransitionCommand = new IntakeTransitionCommand(transitionSubsystem, intakeSubsystem);
    ampVisionCommand = new AmpVisionCommand(driveSubsystem);
    speakerVisionCommand = new SpeakerVisionCommand(driveSubsystem);
    trapVisionCommand = new TrapVisionCommand(driveSubsystem);

    blueTwoPiece = new BlueTwoPieceCommand(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem);
    redTwoPiece = new RedTwoPieceCommand(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem);
    blueFourPiece = new BlueFourPieceCommand(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem);
    redFourPiece = new RedFourPieceCommand(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem);
    blueCenterLineThreePiece = new BlueCenterLineThreePieceCommand(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem);
    redCenterLineThreePiece = new RedCenterLineThreePieceCommand(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem);
    blueCenterLineFourPiece = new BlueCenterLineFourPieceCommand(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem);
    redCenterLineFourPiece = new RedCenterLineFourPieceCommand(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem);
    flipBlueCenterLineFourPiece = new FlipBlueCenterLineFourPieceCommand(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem);
    flipRedCenterLineFourPiece = new FlipRedCenterLineFourPieceCommand(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem);
    blueFivePiece = new BlueFivePieceCommand(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem);
    redFivePiece = new RedFivePieceCommand(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem);

    driveSubsystem.setDefaultCommand(driveCommand);

    robotState = RobotStates.DRIVE;
    climberState = ClimberStates.PREPARE_CLIMB;

    autoChooser = new SendableChooser<>();

    autoChooser.setDefaultOption("Blue 2 Piece", blueTwoPiece);
    autoChooser.addOption("Red 2 Piece", redTwoPiece);
    autoChooser.addOption("Blue Wing 4 Piece", blueFourPiece);
    autoChooser.addOption("Red Wing 4 Piece", redFourPiece);
    autoChooser.addOption("Blue Center Line Stage 3 Piece", blueCenterLineThreePiece);
    autoChooser.addOption("Red Center Line Stage 3 Piece", redCenterLineThreePiece);
    autoChooser.addOption("Blue Center Line 4 Piece", blueCenterLineFourPiece);
    autoChooser.addOption("Red Center Line 4 Piece", redCenterLineFourPiece);
    autoChooser.addOption("Blue Center Line Side 3 Piece", flipBlueCenterLineFourPiece);
    autoChooser.addOption("Red Center Line Side 3 Piece", flipRedCenterLineFourPiece);
    autoChooser.addOption("Blue 5 Piece", blueFivePiece);
    autoChooser.addOption("Red 5 Piece", redFivePiece);

    // Configure the trigger bindings
    configureBindings();

    Shuffleboard.getTab("Competition").add("Auto Chooser", autoChooser);
    //Shuffleboard.getTab("Competition").add("Robot State", state);
    Shuffleboard.getTab("Competition").addString("Robot State", () -> robotState.name());
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

    // Driver Controller: drive controls (left/right joystick), barf (left trigger - set to run as a button), intake (left bumper), score (right bumper), vision align (Y)
    // Operator Controller: change scoring location (4 states), A - amp, X - climb, Y - trap, B - speaker, LT - LED yellow, RT - LED purple

    // Vision Align \\
    driverController.y().toggleOnTrue(new SelectCommand<RobotStates>(Map.ofEntries(
      Map.entry(RobotStates.AMP, new AmpVisionCommand(driveSubsystem)),
      Map.entry(RobotStates.SPEAKER, new SpeakerVisionCommand(driveSubsystem)),
      Map.entry(RobotStates.TRAP, new TrapVisionCommand(driveSubsystem))),
    () -> robotState));

    // INTAKE STATE \\
    driverController.leftBumper().toggleOnTrue(intakeTransitionCommand);

    // BARF STATE \\ 
    driverController.leftTrigger().onTrue(new InstantCommand(() -> robotState = RobotStates.BARF));
    driverController.leftTrigger().whileTrue(new RunCommand(() -> intakeSubsystem.setIntakeVoltage(IntakeConstants.INTAKE_OUT_SPEED), intakeSubsystem));
    driverController.leftTrigger().whileTrue(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(-1.0 * TransitionConstants.TRANSITION_SPEED)));
    driverController.leftTrigger().onFalse(new InstantCommand(() -> intakeSubsystem.stopIntake()));
    driverController.leftTrigger().onFalse(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(0)));
    driverController.leftTrigger().onFalse(new InstantCommand(() -> robotState = RobotStates.DRIVE));

    // Scores
    driverController.rightBumper().onTrue(new SelectCommand<RobotStates>(Map.ofEntries(
      Map.entry(RobotStates.AMP, new ParallelCommandGroup(new InstantCommand(() -> shooterSubsystem.stopShooterMotor()), 
          new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES)))
        .andThen(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_AMP_POSITION, 0))
        .andThen(new WaitCommand(0.3))
        .andThen(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.AMP_TRANSITION_SPEED)))
        .andThen(new WaitCommand(0.4))
        .andThen(new InstantCommand(() -> transitionSubsystem.stopTransition()))
        .andThen(new InstantCommand(() -> shooterSubsystem.stopShooterMotor()))
        .andThen(new InstantCommand(() -> LEDSubsystem.setColor(LEDColors.OFF)))
        .andThen(new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES)))
        .andThen(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT, 0))
        .andThen(new InstantCommand(() -> robotState = RobotStates.DRIVE))),
      Map.entry(RobotStates.SPEAKER, new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> shooterAngleSubsystem.getTargetAngle()))
        .andThen(new WaitCommand(0.1))
        .andThen(new ShooterRevUpCommand(shooterSubsystem))
        .andThen(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED)))
        .andThen(new WaitCommand(0.4))
        .andThen(new InstantCommand(() -> transitionSubsystem.stopTransition()))
        .andThen(new InstantCommand(() -> shooterSubsystem.stopShooterMotor()))
        .andThen(new InstantCommand(() -> LEDSubsystem.setColor(LEDColors.OFF)))
        .andThen(new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES)))
        .andThen(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT, 0))
        .andThen(new InstantCommand(() -> robotState = RobotStates.DRIVE)))),
      () -> robotState));

    // AMP STATE \\
    operatorController.a().onTrue(new InstantCommand(() -> robotState = RobotStates.AMP));
    
    // SHOOT STATE \\
    operatorController.b().onTrue(new InstantCommand(() -> robotState = RobotStates.SPEAKER));
    
    // CLIMB STATE \\
    operatorController.x().onTrue(new ClimberDanceCommand(climberSubsystem, armSubsystem, transitionSubsystem));

    // SCORE IN TRAP \\
    operatorController.y().onTrue(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.AMP_TRANSITION_SPEED))
        .andThen(new WaitCommand(0.4))
        .andThen(new InstantCommand(() -> transitionSubsystem.stopTransition())));


    // Testing Controller
    // testingcontroller.rightBumper().onTrue(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED))
    //   .andThen(new InstantCommand(() -> LEDSubsystem.setColor(LEDColors.OFF)))
    //   .andThen(new WaitCommand(0.5))
    //   .andThen(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(0))
    //   .andThen(new InstantCommand(() -> shooterSubsystem.setShooterMotorVelocity(0))
    //   .andThen(new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES))
    //   .andThen(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT, 0)))
    //   .andThen(new InstantCommand(() -> robotState = RobotStates.DRIVE)))));

    // testingcontroller.a().onTrue(new InstantCommand(() -> armSubsystem.setArmMotorPosition(90)));

    // testingcontroller.leftBumper().toggleOnTrue(intakeTransitionCommand);

    // testingcontroller.leftTrigger().onTrue(new InstantCommand(() -> robotState = RobotStates.BARF));
    // testingcontroller.leftTrigger().whileTrue(new RunCommand(() -> intakeSubsystem.setIntakeVoltage(IntakeConstants.INTAKE_OUT_SPEED), intakeSubsystem));
    // testingcontroller.leftTrigger().whileTrue(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(-1.0 * TransitionConstants.TRANSITION_SPEED)));
    // testingcontroller.leftTrigger().onFalse(new InstantCommand(() -> intakeSubsystem.stopIntake()));
    // testingcontroller.leftTrigger().onFalse(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(0)));
    // testingcontroller.leftTrigger().onFalse(new InstantCommand(() -> robotState = RobotStates.DRIVE));

    // testingcontroller.x().toggleOnTrue(shooterTuningCommand);
    // testingcontroller.b().onTrue((new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED)))
    //     .andThen(new WaitCommand(0.6))
    //     .andThen(new InstantCommand(() -> transitionSubsystem.stopTransition())));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  /**
   * Resets shooter to start teleop
   */
  public void resetShooter() {
    shooterSubsystem.stopShooterMotor();
    shooterAngleSubsystem.setAngle(() -> AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES);
    transitionSubsystem.stopTransition();
    intakeSubsystem.stopIntake();
  }

  /**
   * Turns LEDs blue when an AprilTag is detected while in amp, speaker, or trap state
   */
  public void setVisionLEDs() {
    if(robotState.equals(RobotStates.AMP) && Cameras.isTarget(Cameras.ampCamera)) {

      LEDSubsystem.setColor(LEDColors.BLUE);

    } else if(robotState.equals(RobotStates.SPEAKER) && Cameras.isTarget(Cameras.speakerCamera)) {

      LEDSubsystem.setColor(LEDColors.BLUE);

    } else if(robotState.equals(RobotStates.TRAP) && Cameras.isTarget(Cameras.ampCamera)) {

      LEDSubsystem.setColor(LEDColors.BLUE);

    } else {

      LEDSubsystem.setColor(LEDColors.OFF);
    }
  }
}

