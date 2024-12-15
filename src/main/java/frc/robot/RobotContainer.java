// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.PowerDistribution;
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
import frc.robot.commands.IntakeTransitionCommand;
import frc.robot.commands.ShooterRevUpCommand;
import frc.robot.commands.ShooterTuningCommand;
import frc.robot.commands.SimulationCommand;
import frc.robot.commands.SpeakerVisionCommand;
import frc.robot.commands.autoCommands.BlueAmpThreePieceCommand;
import frc.robot.commands.autoCommands.BlueCenterLineFourPieceCommand;
import frc.robot.commands.autoCommands.BlueCenterLineThreePieceCommand;
import frc.robot.commands.autoCommands.BlueFivePieceCommand;
import frc.robot.commands.autoCommands.BlueFourPieceCommand;
import frc.robot.commands.autoCommands.BlueTwoPieceCommand;
import frc.robot.commands.autoCommands.FlipBlueCenterLineFourPieceCommand;
import frc.robot.commands.autoCommands.FlipRedCenterLineFourPieceCommand;
import frc.robot.commands.autoCommands.RedAmpThreePieceCommand;
import frc.robot.commands.autoCommands.RedCenterLineFourPieceCommand;
import frc.robot.commands.autoCommands.RedCenterLineThreePieceCommand;
import frc.robot.commands.autoCommands.RedFivePieceCommand;
import frc.robot.commands.autoCommands.RedFourPieceCommand;
import frc.robot.commands.autoCommands.RedTwoPieceCommand;
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
  private final PowerDistribution pdh;
  
  // Controllers \\
  private final CommandXboxController driverController; 
  private final CommandXboxController operatorController;
  private final CommandXboxController simulationController;
  //private final CommandXboxController testingcontroller;

  private static final int driverControllerPort = 0;
  private static final int operatorControllerPort = 1;
  //private static final int testingcontrollerPort = 2;
  private static final int simulationControllerPort = 3;

  // Subsystems \\
  private final DriveSubsystem driveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final TransitionSubsystem transitionSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ShooterAngleSubsystem shooterAngleSubsystem;
  private final TransitionArmSubsystem armSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final Telemetry telemetry;
  private final Cameras cameras;

  // Commands \\
  private final DriveCommand driveCommand;
  private final IntakeTransitionCommand intakeTransitionCommand;

  private final BlueTwoPieceCommand blueTwoPiece;
  private final RedTwoPieceCommand redTwoPiece;
  private final BlueFourPieceCommand blueFourPiece;
  private final RedFourPieceCommand redFourPiece;
  private final BlueCenterLineThreePieceCommand blueCenterLineThreePiece;
  private final RedCenterLineThreePieceCommand redCenterLineThreePiece;
  private final BlueCenterLineFourPieceCommand blueCenterLineFourPiece;
  private final RedCenterLineFourPieceCommand redCenterLineFourPiece;
  private final FlipBlueCenterLineFourPieceCommand flipBlueCenterLineFourPiece;
  private final FlipRedCenterLineFourPieceCommand flipRedCenterLineFourPiece;
  private final BlueFivePieceCommand blueFivePiece;
  private final RedFivePieceCommand redFivePiece;
  private final BlueAmpThreePieceCommand blueAmpThreePiece;
  private final RedAmpThreePieceCommand redAmpThreePiece;

  private final ShooterTuningCommand shooterTuningCommand;

  
  public static RobotStates robotState;
  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Comment this line out to use TunerX, uncomment to improve CAN utilization
    Unmanaged.setPhoenixDiagnosticsStartTime(-1);

    pdh = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

    driverController = new CommandXboxController(driverControllerPort);
    operatorController = new CommandXboxController(operatorControllerPort);
    simulationController = new CommandXboxController(simulationControllerPort);
    //testingController = new CommandXboxController(testingcontrollerPort);

    DriverStation.silenceJoystickConnectionWarning(true);
    pdh.clearStickyFaults();

    driveSubsystem = new DriveSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    transitionSubsystem = new TransitionSubsystem();
    armSubsystem = new TransitionArmSubsystem();
    ledSubsystem = new LEDSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    shooterAngleSubsystem = new ShooterAngleSubsystem();
    telemetry = new Telemetry(0);
    cameras = new Cameras();

    driveCommand = new DriveCommand(driveSubsystem, driverController);
    intakeTransitionCommand = new IntakeTransitionCommand(transitionSubsystem, intakeSubsystem);

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
    blueAmpThreePiece = new BlueAmpThreePieceCommand(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem, armSubsystem);
    redAmpThreePiece = new RedAmpThreePieceCommand(driveSubsystem, intakeSubsystem, transitionSubsystem, shooterSubsystem, shooterAngleSubsystem, armSubsystem);

    shooterTuningCommand = new ShooterTuningCommand(shooterAngleSubsystem, shooterSubsystem);

    driveSubsystem.setDefaultCommand(driveCommand);

    robotState = RobotStates.DRIVE;
    autoChooser = new SendableChooser<>();

    // autoChooser.setDefaultOption("Blue 2 Piece", blueTwoPiece);
    // autoChooser.addOption("Red 2 Piece", redTwoPiece);
    autoChooser.addOption("Blue Wing 4 Piece", blueFourPiece);
    autoChooser.addOption("Red Wing 4 Piece", redFourPiece);
    autoChooser.addOption("Blue Center Line Stage 3 Piece", blueCenterLineThreePiece);
    autoChooser.addOption("Red Center Line Stage 3 Piece", redCenterLineThreePiece);
    // autoChooser.addOption("Blue Center Line 4 Piece", blueCenterLineFourPiece);
    // autoChooser.addOption("Red Center Line 4 Piece", redCenterLineFourPiece);
    autoChooser.addOption("Blue Skip Wing 4 Piece", flipBlueCenterLineFourPiece);
    autoChooser.addOption("Red Skip Wing 4 Piece", flipRedCenterLineFourPiece);
    // autoChooser.addOption("Blue 5 Piece", blueFivePiece);
    // autoChooser.addOption("Red 5 Piece", redFivePiece);
    autoChooser.addOption("Blue Amp", blueAmpThreePiece);
    autoChooser.addOption("Red Amp", redAmpThreePiece);


    // Telemetry
    driveSubsystem.registerTelemetry(telemetry::telemeterize);

    // Configure the trigger bindings
    configureBindings();

    Shuffleboard.getTab("Competition").add("Auto Chooser", autoChooser);
    //Shuffleboard.getTab("Competition").add("Robot State", state);
    Shuffleboard.getTab("Competition").addString("Robot State", () -> robotState.name());
    // Shuffleboard.getTab("Competition").addString("Climber State", () -> climberState.name());
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
    // Operator Controller: change scoring location (3 states), A - amp, Y - shuttle, B - speaker, X - actuate climb

    // Vision Align \\
    driverController.rightBumper().toggleOnTrue(new SelectCommand<RobotStates>(Map.ofEntries(
      Map.entry(RobotStates.AMP, new AmpVisionCommand(driveSubsystem)),
      Map.entry(RobotStates.SPEAKER, new SpeakerVisionCommand(driveSubsystem)),
      Map.entry(RobotStates.SHUTTLE, new InstantCommand(() -> robotState = RobotStates.SHUTTLE)),
      Map.entry(RobotStates.SUBWOOFER, new InstantCommand(() -> robotState = RobotStates.SUBWOOFER)),
      Map.entry(RobotStates.DRIVE, new InstantCommand(() -> robotState = RobotStates.DRIVE)),
      Map.entry(RobotStates.BARF, new InstantCommand(() -> robotState = RobotStates.BARF)),
      Map.entry(RobotStates.INTAKE, new InstantCommand(() -> robotState = RobotStates.INTAKE))),
    () -> robotState));

    // INTAKE STATE \\
    driverController.leftBumper().toggleOnTrue(intakeTransitionCommand);

    // BARF STATE \\ 
    driverController.leftTrigger().onTrue(new InstantCommand(() -> robotState = RobotStates.BARF));
    driverController.leftTrigger().whileTrue(new RunCommand(() -> intakeSubsystem.setIntakeVoltage(IntakeConstants.INTAKE_OUT_SPEED), intakeSubsystem));
    driverController.leftTrigger().whileTrue(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(-1.0 * TransitionConstants.TRANSITION_SPEED), transitionSubsystem));
    driverController.leftTrigger().onFalse(new InstantCommand(() -> intakeSubsystem.stopIntake(), intakeSubsystem));
    driverController.leftTrigger().onFalse(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(0), transitionSubsystem));
    driverController.leftTrigger().onFalse(new InstantCommand(() -> robotState = RobotStates.DRIVE));

    // Scores
    driverController.rightTrigger().onTrue(new SelectCommand<RobotStates>(Map.ofEntries(
      Map.entry(RobotStates.AMP, new ParallelCommandGroup(new InstantCommand(() -> shooterSubsystem.stopShooterMotor(), shooterSubsystem), 
          new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES), shooterAngleSubsystem))
        .andThen(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_AMP_POSITION, 0))
        .andThen(new WaitCommand(0.3))
        .andThen(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.AMP_TRANSITION_SPEED), transitionSubsystem))
        .andThen(new WaitCommand(0.4))
        .andThen(new InstantCommand(() -> transitionSubsystem.stopTransition(), transitionSubsystem))
        .andThen(new InstantCommand(() -> shooterSubsystem.stopShooterMotor(), shooterSubsystem))
        .andThen(new InstantCommand(() -> LEDSubsystem.setColor(LEDColors.OFF)))
        .andThen(new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES), shooterAngleSubsystem))
        .andThen(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT, 0))
        .andThen(new InstantCommand(() -> robotState = RobotStates.DRIVE))),
      Map.entry(RobotStates.SPEAKER, new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> shooterAngleSubsystem.getTargetAngle()), shooterAngleSubsystem)
        .andThen(new WaitCommand(0.1))
        .andThen(new InstantCommand (() -> shooterSubsystem.setShooterMotorVelocity(5400)))
        .andThen(new WaitCommand(0.8))
        .andThen(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED), transitionSubsystem))
        .andThen(new WaitCommand(0.4))
        .andThen(new InstantCommand(() -> transitionSubsystem.stopTransition(), transitionSubsystem))
        .andThen(new InstantCommand(() -> shooterSubsystem.stopShooterMotor(), shooterSubsystem))
        .andThen(new InstantCommand(() -> LEDSubsystem.setColor(LEDColors.OFF)))
        .andThen(new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES), shooterAngleSubsystem))
        .andThen(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT, 0))
        .andThen(new InstantCommand(() -> robotState = RobotStates.DRIVE))),
      Map.entry(RobotStates.SHUTTLE, new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> shooterAngleSubsystem.getTargetAngle()), shooterAngleSubsystem)
        .andThen(new WaitCommand(0.1))
        .andThen(new ShooterRevUpCommand(shooterSubsystem, 0))
        .andThen(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED), transitionSubsystem))
        .andThen(new WaitCommand(0.4))
        .andThen(new InstantCommand(() -> transitionSubsystem.stopTransition(), transitionSubsystem))
        .andThen(new InstantCommand(() -> shooterSubsystem.stopShooterMotor(), shooterSubsystem))
        .andThen(new InstantCommand(() -> LEDSubsystem.setColor(LEDColors.OFF)))
        .andThen(new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES), shooterAngleSubsystem))
        .andThen(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT, 0))
        .andThen(new InstantCommand(() -> robotState = RobotStates.DRIVE))),
      Map.entry(RobotStates.SUBWOOFER, new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> Constants.TrajectoryConstants.DEGREE_AT_SUBWOOFER), shooterAngleSubsystem)
        .andThen(new WaitCommand(0.1))
        .andThen(new ShooterRevUpCommand(shooterSubsystem, 0))
        .andThen(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED), transitionSubsystem))
        .andThen(new WaitCommand(0.4))
        .andThen(new InstantCommand(() -> transitionSubsystem.stopTransition(), transitionSubsystem))
        .andThen(new InstantCommand(() -> shooterSubsystem.stopShooterMotor(), shooterSubsystem))
        .andThen(new InstantCommand(() -> LEDSubsystem.setColor(LEDColors.OFF)))
        .andThen(new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES), shooterAngleSubsystem))
        .andThen(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT, 0))
        .andThen(new InstantCommand(() -> robotState = RobotStates.DRIVE))),
      Map.entry(RobotStates.DRIVE, new InstantCommand(() -> robotState = RobotStates.DRIVE)),
      Map.entry(RobotStates.BARF, new InstantCommand(() -> robotState = RobotStates.BARF)),
      Map.entry(RobotStates.INTAKE, new InstantCommand(() -> robotState = RobotStates.INTAKE))),
      () -> robotState));

    // AMP STATE \\
    driverController.a().onTrue(new InstantCommand(() -> robotState = RobotStates.AMP)
            .andThen(new InstantCommand(() -> LEDSubsystem.setColor(LEDColors.RED))));

    // SHOOT STATE \\
    driverController.b().onTrue(new InstantCommand(() -> robotState = RobotStates.SPEAKER)
            .andThen(new InstantCommand(() -> LEDSubsystem.setColor(LEDColors.BLUE))));

    // SHUTTLE STATE \\
    driverController.y().onTrue(new InstantCommand(() -> robotState = RobotStates.SHUTTLE));

    // Blocker up \\
    operatorController.b().onTrue(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_AMP_POSITION - 2, 0));

    // SUBWOOFER STATE \\
    driverController.x().onTrue(new InstantCommand(() -> robotState = RobotStates.SUBWOOFER)
              .andThen(new InstantCommand(() -> LEDSubsystem.setColor(LEDColors.PURPLE))));

    // GYRO RESET \\
    operatorController.y().onTrue(new InstantCommand(() -> driveSubsystem.getPigeon2().reset()));

    // Shooter testing \\ TODO Remove when done
    // driverController.leftTrigger().whileTrue(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED)));
    // driverController.leftTrigger().onFalse(new InstantCommand(() -> transitionSubsystem.stopTransition()));

    // Testing Controller
    // testingcontroller.rightBumper().onTrue(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED))
    //   .andThen(new InstantCommand(() -> LEDSubsystem.setColor(LEDColors.OFF)))
    //   .andThen(new WaitCommand(0.5))
    //   .andThen(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(0))
    //   .andThen(new InstantCommand(() -> shooterSubsystem.setShooterMotorVelocity(0), shooterSubsystem)
    //   .andThen(new InstantCommand(() -> shooterAngleSubsystem.setAngle(() -> AngleConstants.MINIMUM_SOFT_LIMIT_DEGREES))
    //   .andThen(new ArmCommand(armSubsystem, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT, 0)))
    //   .andThen(new InstantCommand(() -> robotState = RobotStates.DRIVE)))));

    //testingcontroller.a().onTrue(new InstantCommand(() -> climberSubsystem.setClimberVoltage(0)));

    // testingcontroller.leftBumper().toggleOnTrue(intakeTransitionCommand);

    // testingcontroller.leftTrigger().onTrue(new InstantCommand(() -> robotState = RobotStates.BARF));
    // testingcontroller.leftTrigger().whileTrue(new RunCommand(() -> intakeSubsystem.setIntakeVoltage(IntakeConstants.INTAKE_OUT_SPEED), intakeSubsystem));
    // testingcontroller.leftTrigger().whileTrue(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(-1.0 * TransitionConstants.TRANSITION_SPEED)));
    // testingcontroller.leftTrigger().onFalse(new InstantCommand(() -> intakeSubsystem.stopIntake(), intakeSubsystem));
    // testingcontroller.leftTrigger().onFalse(new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(0)));
    // testingcontroller.leftTrigger().onFalse(new InstantCommand(() -> robotState = RobotStates.DRIVE));

    //testingController.x().toggleOnTrue(shooterTuningCommand);
    // testingcontroller.x().onFalse(new InstantCommand(() -> climberSubsystem.setClimberVoltage(0)));
    // testingcontroller.b().onTrue((new InstantCommand(() -> transitionSubsystem.setTransitionVoltage(TransitionConstants.TRANSITION_SPEED)))
    //     .andThen(new WaitCommand(0.6))
    //     .andThen(new InstantCommand(() -> transitionSubsystem.stopTransition())));

    // Simulation controller
    simulationController.b().onTrue(new SimulationCommand(armSubsystem, TransitionArmConstants.ARM_AMP_POSITION, 6)
    .andThen(new WaitCommand(.75))
    .andThen(new SimulationCommand(armSubsystem, TransitionArmConstants.ARM_REVERSE_SOFT_LIMIT, -6)));
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

    if(robotState.equals(RobotStates.AMP) && (Cameras.isTarget(Cameras.ampCamera) || Cameras.isTarget(Cameras.speakerCamera))) {

      LEDSubsystem.setColor(LEDColors.BLUE);

    } else if(robotState.equals(RobotStates.SPEAKER) && Cameras.isTarget(Cameras.speakerCamera)) {

      LEDSubsystem.setColor(LEDColors.BLUE);

    } else {

      LEDSubsystem.setColor(LEDColors.OFF);

    }
  }
}

