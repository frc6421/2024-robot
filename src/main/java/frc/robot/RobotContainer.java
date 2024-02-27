// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AmpAlignVisionCommand;
import frc.robot.commands.AutoTestCommand;
import frc.robot.commands.BlueCenterLineThreePieceCommand;
import frc.robot.commands.BlueFourPieceCommand;
import frc.robot.commands.BlueTwoPieceCommand;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransitionArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;
import frc.robot.subsystems.ShooterAngleSubsystem.AngleConstants;
import frc.robot.subsystems.TransitionArmSubsystem.TransitionArmConstants;
import frc.robot.subsystems.TransitionSubsystem.TransitionConstants;

import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.unmanaged.Unmanaged;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDConstants.LEDColors;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.commands.IntakeTransitionCommand;
import frc.robot.commands.RedFourPieceCommand;
import frc.robot.commands.RedTwoPieceCommand;
import frc.robot.Constants.RobotStates;
import frc.robot.commands.ShooterRevUpCommand;
import frc.robot.commands.SpeakerAlignVisionCommand;
import frc.robot.commands.SpeakerDistanceCommand;
import frc.robot.commands.VisionTestCommand;
//import frc.robot.subsystems.CANdleSubsystem;
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

  private static final int driverControllerPort = 0;
  private static final int operatorControllerPort = 1;

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

  private final VisionTestCommand visionTestCommand;
  private final AmpAlignVisionCommand ampAlignVisionCommand;
  private final SpeakerAlignVisionCommand speakerAlignVisionCommand;
  private final SpeakerDistanceCommand speakerDistanceCommand;

  //private final AutoTestCommand autoTest;
  BlueTwoPieceCommand blueTwoPiece;
  RedTwoPieceCommand redTwoPiece;
  BlueFourPieceCommand blueFourPiece;
  RedFourPieceCommand redFourPiece;
  BlueCenterLineThreePieceCommand blueCenterLineThreePiece;
  
  public static RobotStates state;

  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Comment this line out to use TunerX, uncomment to improve CAN utilization
    Unmanaged.setPhoenixDiagnosticsStartTime(-1);

    driverController = new CommandXboxController(driverControllerPort);
    operatorController = new CommandXboxController(operatorControllerPort);

    DriverStation.silenceJoystickConnectionWarning(true);

    driveSubsystem = new DriveSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    transitionSubsystem = new TransitionSubsystem();
    armSubsystem = new TransitionArmSubsystem();
    climberSubsystem = new ClimberSubsystem();

    driveCommand = new DriveCommand(driveSubsystem, driverController);
    shooterSubsystem = new ShooterSubsystem();
    shooterAngleSubsystem = new ShooterAngleSubsystem();
    ledSubsystem = new LEDSubsystem();

    intakeTransitionCommand = new IntakeTransitionCommand(transitionSubsystem, intakeSubsystem);
    ampAlignVisionCommand = new AmpAlignVisionCommand(driveSubsystem);
    speakerAlignVisionCommand = new SpeakerAlignVisionCommand(driveSubsystem);
    speakerDistanceCommand = new SpeakerDistanceCommand(driveSubsystem);

    visionTestCommand = new VisionTestCommand(driveSubsystem);

    driveSubsystem.setDefaultCommand(driveCommand);

    //autoTest = new AutoTestCommand(driveSubsystem);
    blueTwoPiece = new BlueTwoPieceCommand(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem);
    redTwoPiece = new RedTwoPieceCommand(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem);
    blueFourPiece = new BlueFourPieceCommand(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem);
    redFourPiece = new RedFourPieceCommand(driveSubsystem, intakeSubsystem);
    blueCenterLineThreePiece = new BlueCenterLineThreePieceCommand(driveSubsystem, intakeSubsystem);

    state = RobotStates.DRIVE;

    autoChooser = new SendableChooser<>();

    autoChooser.setDefaultOption("Blue 2 Piece", blueTwoPiece);
    autoChooser.addOption("Red 2 Piece", redTwoPiece);
    autoChooser.addOption("Blue 4 Piece", blueFourPiece);
    autoChooser.addOption("Red 4 Piece", redFourPiece);
    

    // Configure the trigger bindings
    configureBindings();

    Shuffleboard.getTab("Competition").add("Auto Chooser", autoChooser);
    //Shuffleboard.getTab("Competition").add("Robot State", state);
    Shuffleboard.getTab("Competition").add("Robot State", state.toString());
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
    // Operator Controller: change scoring location (4 states), A - amp, X - climb, Y - trap, B - speaker, LT - LED yellow, RT - LED purple, 

    // INTAKE STATE \\
    driverController.leftBumper().toggleOnTrue(intakeTransitionCommand);

    // BARF STATE \\ 
    driverController.leftTrigger().onTrue(new InstantCommand(() -> state = RobotStates.BARF));
    driverController.leftTrigger().whileTrue(new RunCommand(() -> intakeSubsystem.setIntakeVoltage(IntakeConstants.INTAKE_OUT_SPEED), intakeSubsystem));
    driverController.leftTrigger().whileTrue(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(-1.0 * TransitionConstants.TRANSITION_SPEED)));
    driverController.leftTrigger().onFalse(new InstantCommand(() -> intakeSubsystem.stopIntake()));
    driverController.leftTrigger().onFalse(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(0)));
    driverController.leftTrigger().onFalse(new InstantCommand(() -> state = RobotStates.DRIVE));

    // Scores
    // TODO Profile for transition arm
    driverController.rightBumper().onTrue(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED))
      .andThen(new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0)))
      .andThen(new WaitCommand(0.5))
      .andThen(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(0))
      .andThen(new InstantCommand(() -> shooterSubsystem.setShooterMotorVelocity(0))
      .andThen(new InstantCommand(() -> shooterAngleSubsystem.setAngle(AngleConstants.MINNIMUM_SOFT_LIMIT_DEGREES))
      .andThen(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT)))
      .andThen(new InstantCommand(() -> state = RobotStates.DRIVE)))));

    //TODO remove testing buttons
    driverController.a().whileTrue(speakerAlignVisionCommand
      .alongWith(speakerDistanceCommand));

    driverController.b().onTrue(new InstantCommand(() -> shooterAngleSubsystem.setAngle(23))
      .alongWith(new InstantCommand(() -> shooterSubsystem.setTopShooterMotorVelocity(4500 - 100)))
      .alongWith(new InstantCommand(() -> shooterSubsystem.setBottomShooterMotorVelocity(4500))));
    driverController.x().onTrue(new InstantCommand(() -> shooterAngleSubsystem.setAngle(AngleConstants.MINNIMUM_SOFT_LIMIT_DEGREES))
      .alongWith(new InstantCommand(() -> shooterSubsystem.stopShooterMotor())));


    // Arm out for AMP
    operatorController.a().onTrue(new InstantCommand(() -> state = RobotStates.AMP)
      .andThen(new InstantCommand(() -> shooterSubsystem.setShooterMotorVelocity(0))
      .andThen(new InstantCommand(() -> shooterAngleSubsystem.setAngle(AngleConstants.MINNIMUM_SOFT_LIMIT_DEGREES))
      .andThen(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_AMP_POSITION)))));
    
    // SHOOT STATE \\

    // Sub
    operatorController.b().whileTrue(new InstantCommand(() -> state = RobotStates.SHOOT)
      .andThen(new ParallelCommandGroup(new ShooterRevUpCommand(shooterSubsystem), new InstantCommand(() -> shooterAngleSubsystem.setAngle(45)))
      .andThen(new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1)))));
    
    // Podium
    operatorController.y().whileTrue(new InstantCommand(() -> state = RobotStates.SHOOT)
      .andThen(new ParallelCommandGroup(new ShooterRevUpCommand(shooterSubsystem), new InstantCommand(() -> shooterAngleSubsystem.setAngle(30)))
      .andThen(new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1)))));
    
    // TODO climber button
    // CLIMB STATE \\
    //operatorController.x().onTrue();

    // TRAP STATE \\ 
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
    shooterAngleSubsystem.setAngle(AngleConstants.MINNIMUM_SOFT_LIMIT_DEGREES);
  }
}

