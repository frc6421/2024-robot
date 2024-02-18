package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class DriveSubsystem extends SwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;
  public SwerveDriveKinematics kinematics;
  public ApplyModuleStates autoDriveRequest;

  // PhotonVision Cameras
  // private PhotonCamera camera1;
  // private PhotonCamera camera2;
  // private PhotonCamera camera3;
  // private PhotonCamera camera4;

  // private PhotonPoseEstimator camera1PoseEstimator;
  // private PhotonPoseEstimator camera2PoseEstimator;
  // private PhotonPoseEstimator camera3PoseEstimator;
  // private PhotonPoseEstimator camera4PoseEstimator;

  public class DriveConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs STEER_GAINS = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
        .withKP(0.156).withKI(0).withKD(0)
        .withKS(0.29).withKV(0.11).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double SLIP_CURRENT_AMPS = 35;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double SPEED_AT_12_VOLTS_METERS_PER_SEC = 4.73;

    // Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double COUPLE_RATIO = 3.5714285714285716;

    private static final double DRIVE_GEAR_RATIO = 6.746031746031747;
    private static final double STEER_GEAR_RATIO = 21.428571428571427;
    //TODO update after initial measurements and before each competition/everytime treads are changed
    private static final double WHEEL_RADIUS_INCHES = 1.91;

    private static final boolean STEER_MOTOR_REVERSED = true;
    private static final boolean INVERT_LEFT_SIDE = false;
    private static final boolean INVERT_RIGHT_SIDE = true;

    private static final int PIGEON_CAN_ID = 18;

    // These are only used for simulation
    private static final double STEER_INERTIA = 0.00001;
    private static final double DRIVE_INERTIA = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double STEER_FRICTION_VOLTAGE = 0.25;
    private static final double DRIVE_FRICTION_VOLTAGE = 0.25;

    private static final SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants()
        .withPigeon2Id(PIGEON_CAN_ID);

    private static final SwerveModuleConstantsFactory constantCreator = new SwerveModuleConstantsFactory()
        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
        .withSteerMotorGearRatio(STEER_GEAR_RATIO)
        .withWheelRadius(WHEEL_RADIUS_INCHES)
        .withSlipCurrent(SLIP_CURRENT_AMPS)
        .withSteerMotorGains(STEER_GAINS)
        .withDriveMotorGains(DRIVE_GAINS)
        .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
        .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
        .withSpeedAt12VoltsMps(SPEED_AT_12_VOLTS_METERS_PER_SEC)
        .withSteerInertia(STEER_INERTIA)
        .withDriveInertia(DRIVE_INERTIA)
        .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
        .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE)
        .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
        .withCouplingGearRatio(COUPLE_RATIO)
        .withSteerMotorInverted(STEER_MOTOR_REVERSED);

    // Front Left
    private static final int FRONT_LEFT_DRIVE_MOTOR_CAN_ID = 12;
    private static final int FRONT_LEFT_STEER_MOTOR_CAN_ID = 13;
    private static final int FRONT_LEFT_CANCODER_CAN_ID = 13;
    private static final double FRONT_LEFT_ENCODER_OFFSET = -0.28809;

    private static final double FRONT_LEFT_X_POS_INCHES = 8.125;
    private static final double FRONT_LEFT_Y_POS_INCHES = 22.75 / 2;

    // Front Right
    private static final int FRONT_RIGHT_DRIVE_MOTOR_CAN_ID = 10;
    private static final int FRONT_RIGHT_STEER_MOTOR_CAN_ID = 11;
    private static final int FRONT_RIGHT_CANCODER_CAN_ID = 11;
    private static final double FRONT_RIGHT_ENCODER_OFFSET = 0.26978;

    private static final double FRONT_RIGHT_X_POS_INCHES = 8.125;
    private static final double FRONT_RIGHT_Y_POS_INCHES = -22.75 / 2;

    // Back Left
    private static final int BACK_LEFT_DRIVE_MOTOR_CAN_ID = 16;
    private static final int BACK_LEFT_STEER_MOTOR_CAN_ID = 17;
    private static final int BACK_LEFT_CANCODER_CAN_ID = 17;
    private static final double BACK_LEFT_ENCODER_OFFSET = -0.37476;

    private static final double BACK_LEFT_X_POS_INCHES = -12.625;
    private static final double BACK_LEFT_Y_POS_INCHES = 22.75 / 2;

    // Back Right
    private static final int BACK_RIGHT_DRIVE_MOTOR_CAN_ID = 14;
    private static final int BACK_RIGHT_STEER_MOTOR_CAN_ID = 15;
    private static final int BACK_RIGHT_CANCODER_CAN_ID = 15;
    private static final double BACK_RIGHT_ENCODER_OFFSET = 0.02148;

    private static final double BACK_RIGHT_X_POS_INCHES = -12.625;
    private static final double BACK_RIGHT_Y_POS_INCHES = -22.75 / 2;

    private static final SwerveModuleConstants frontLeft = constantCreator.createModuleConstants(
        FRONT_LEFT_STEER_MOTOR_CAN_ID, FRONT_LEFT_DRIVE_MOTOR_CAN_ID, FRONT_LEFT_CANCODER_CAN_ID, FRONT_LEFT_ENCODER_OFFSET,
        Units.inchesToMeters(FRONT_LEFT_X_POS_INCHES), Units.inchesToMeters(FRONT_LEFT_Y_POS_INCHES), INVERT_LEFT_SIDE);
    private static final SwerveModuleConstants frontRight = constantCreator.createModuleConstants(
        FRONT_RIGHT_STEER_MOTOR_CAN_ID, FRONT_RIGHT_DRIVE_MOTOR_CAN_ID, FRONT_RIGHT_CANCODER_CAN_ID, FRONT_RIGHT_ENCODER_OFFSET,
        Units.inchesToMeters(FRONT_RIGHT_X_POS_INCHES), Units.inchesToMeters(FRONT_RIGHT_Y_POS_INCHES), INVERT_RIGHT_SIDE);
    private static final SwerveModuleConstants backLeft = constantCreator.createModuleConstants(
        BACK_LEFT_STEER_MOTOR_CAN_ID, BACK_LEFT_DRIVE_MOTOR_CAN_ID, BACK_LEFT_CANCODER_CAN_ID, BACK_LEFT_ENCODER_OFFSET,
        Units.inchesToMeters(BACK_LEFT_X_POS_INCHES), Units.inchesToMeters(BACK_LEFT_Y_POS_INCHES), INVERT_LEFT_SIDE);
    private static final SwerveModuleConstants backRight = constantCreator.createModuleConstants(
        BACK_RIGHT_STEER_MOTOR_CAN_ID, BACK_RIGHT_DRIVE_MOTOR_CAN_ID, BACK_RIGHT_CANCODER_CAN_ID, BACK_RIGHT_ENCODER_OFFSET,
        Units.inchesToMeters(BACK_RIGHT_X_POS_INCHES), Units.inchesToMeters(BACK_RIGHT_Y_POS_INCHES), INVERT_RIGHT_SIDE);
    public static final double DRIVE_SLEW_RATE = 7.5;
  }


  public DriveSubsystem() {
    super(DriveConstants.drivetrainConstants, 
      DriveConstants.frontLeft,
      DriveConstants.frontRight,
      DriveConstants.backLeft,
      DriveConstants.backRight);

      kinematics = m_kinematics;

      autoDriveRequest = new ApplyModuleStates();

      ParentDevice.optimizeBusUtilizationForAll(
        getModule(0).getDriveMotor(),
        getModule(0).getSteerMotor(),
        getModule(0).getCANcoder(),
        getModule(1).getDriveMotor(),
        getModule(1).getSteerMotor(),
        getModule(1).getCANcoder(),
        getModule(2).getDriveMotor(),
        getModule(2).getSteerMotor(),
        getModule(2).getCANcoder(),
        getModule(3).getDriveMotor(),
        getModule(3).getSteerMotor(),
        getModule(3).getCANcoder(),
        getPigeon2()
        );

    if (Utils.isSimulation()) {
      startSimThread();
    }

    // Back left camera
    //camera3 = new PhotonCamera("Camera_1_OV9281_USB_Camera");
    // Back right camera
    //camera4 = new PhotonCamera("Camera_6_OV9281_USB_Camera");

    // camera1PoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
    //     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //     camera1,
    //     new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)));

    // camera2PoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
    //     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //     camera2,
    //     new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)));

    // camera3PoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
    //     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //     camera3,
    //     new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)));

    // camera4PoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
    //     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //     camera4,
    //     new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)));
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  public class ApplyModuleStates implements SwerveRequest {
    public SwerveModuleState[] States = new SwerveModuleState[] {};

    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.Velocity;

    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
      var states = States;
      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
      }

      return StatusCode.OK;
    }
    
    public ApplyModuleStates withModuleStates(SwerveModuleState[] state) {
      this.States = state;
      return this;
    }

    public ApplyModuleStates withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }

    public ApplyModuleStates withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }

  public void autoSetModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC);

    autoDriveRequest.withModuleStates(desiredStates);
    autoDriveRequest.apply(m_requestParameters, Modules);
  }

  public Pose2d getPose2d() {
    return getState().Pose;
  }

  @Override
  public void periodic() {

    // Optional<EstimatedRobotPose> pose1 = updatePhotonPoseEstimator(camera1PoseEstimator);
    // Optional<EstimatedRobotPose> pose2 = updatePhotonPoseEstimator(camera2PoseEstimator);
    // Optional<EstimatedRobotPose> pose3 = updatePhotonPoseEstimator(camera3PoseEstimator);
    // Optional<EstimatedRobotPose> pose4 = updatePhotonPoseEstimator(camera4PoseEstimator);

    //TODO determine if we need to reject bad vision pose estimates
    // if(pose1.isPresent()) {

    //   addVisionMeasurement(pose1.get().estimatedPose.toPose2d(),
    //       pose1.get().timestampSeconds);
      
    // }

    // if(pose2.isPresent()) {

    //   addVisionMeasurement(pose2.get().estimatedPose.toPose2d(),
    //       pose2.get().timestampSeconds);
      
    // }

    // if(pose3.isPresent()) {

    //   addVisionMeasurement(pose3.get().estimatedPose.toPose2d(),
    //       pose3.get().timestampSeconds);
      
    // }

    // if(pose4.isPresent()) {

    //   addVisionMeasurement(pose4.get().estimatedPose.toPose2d(),
    //       pose4.get().timestampSeconds);
      
    // }

  }

  private Optional<EstimatedRobotPose> updatePhotonPoseEstimator(PhotonPoseEstimator poseEstimator) {
    return poseEstimator.update();
  }

  /**
   * Gets the estimated pose 
   * @return estimated pose from pose estimator (Pose2d)
   */
  public Pose2d getCurrentPose2d() {
    return m_odometry.getEstimatedPosition();
  }
}
