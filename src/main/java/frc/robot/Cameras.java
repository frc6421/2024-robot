// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Includes camera setup and all methods for getting camera data */
public class Cameras implements Sendable{

    public static PhotonCamera ampCamera = new PhotonCamera("Camera6");
    public static PhotonCamera speakerCamera = new PhotonCamera("Camera1");
    //public static PhotonCamera noteCamera = new PhotonCamera("Camera7");

    public final static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    
    
    private final static Translation2d redSpeakerTranslation = aprilTagFieldLayout.getTagPose(4).get().getTranslation().toTranslation2d();
    private final static Translation2d blueSpeakerTranslation = aprilTagFieldLayout.getTagPose(7).get().getTranslation().toTranslation2d();

    private final static Transform3d robotToAmpCamera = new Transform3d(
        new Translation3d(VisionConstants.AMP_CAMERA_X, VisionConstants.AMP_CAMERA_Y, VisionConstants.AMP_CAMERA_Z),
        new Rotation3d(0, VisionConstants.AMP_CAMERA_PITCH, VisionConstants.AMP_CAMERA_YAW)
    );

    private final static Transform3d robotToSpeakerCamera = new Transform3d(
        new Translation3d(VisionConstants.SPEAKER_CAMERA_X, VisionConstants.SPEAKER_CAMERA_Y, VisionConstants.SPEAKER_CAMERA_Z),
        new Rotation3d(0, VisionConstants.SPEAKER_CAMERA_PITCH, VisionConstants.SPEAKER_CAMERA_YAW)
    );

    public final static PhotonPoseEstimator ampCameraPoseEstimator = new PhotonPoseEstimator(
      aprilTagFieldLayout, 
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      ampCamera, 
      robotToAmpCamera
    );

    public final static PhotonPoseEstimator speakerCameraPoseEstimator = new PhotonPoseEstimator(
      aprilTagFieldLayout, 
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      speakerCamera, 
      robotToSpeakerCamera
    );

    private static double ampCameraEstimatedYaw = 0;
    private static double ampCameraEstimatedPitch = 0;

    private static double speakerCameraEstimatedYaw = 0;
    private static double speakerCameraEstimatedPitch = 0;

    public static double[] ampCameraPose = {0,0,0,0};
    public static double[] speakerCameraPose = {0,0,0,0};

    public static double[] previousAmpCameraPose = {0,0,0,0};
    public static double[] previousSpeakerCameraPose = {0,0,0,0};

    public static Pose3d ampPose3d = new Pose3d();
    public static Pose3d speakerPose3d = new Pose3d();

    public static double ampTimeStamp = 0;
    public static double speakerTimeStamp = 0;

    public Cameras() {
      SendableRegistry.add(this, "Cameras");

      Shuffleboard.getTab("Cameras").add(this);
      DataLogManager.log("tag 4 " + redSpeakerTranslation.toString());
      DataLogManager.log("tag 7 " + blueSpeakerTranslation.toString());
    }

    public static boolean isTarget(PhotonCamera camera) {
        return camera.getLatestResult().hasTargets();
    }

    /**
     * Returns an array with all targets currently detected by the camera
     * 
     * @param camera camera to use
     * @return Object[] list of all targets
     */
    public static Object[] getListOfTargets(PhotonCamera camera) {
        return camera.getLatestResult().getTargets().toArray();
    }

    /**
     * Used with a for loop to check if a desired tag ID is included in the list of targets
     * 
     * @param camera camera to use
     * @param i array index to check
     * @return tagID at that array index
     */
    public static int getTagIDFromList(PhotonCamera camera, int i) {

        return camera.getLatestResult().getTargets().get(i).getFiducialId();

    }

    /**
     * Returns yaw angle of the target in the camera space
     * 
     * @param camera camera to use
     * @param targetTagID desired AprilTag
     * @return yaw angle of target in degrees
     */
    public static double getYaw(PhotonCamera camera, int targetTagID) {
        double yaw = 180.0;
        var result = camera.getLatestResult();

        if(result.hasTargets()) {

            var targets = result.getTargets();

            for(int i = 0; i < targets.size(); i++) {

                if(targets.get(i).getFiducialId() == targetTagID) {

                    yaw = targets.get(i).getYaw();

                }

            }

        }
        switch (camera.getName()) {
          case "Camera6":
            ampCameraEstimatedYaw = yaw;
            break;
          case "Camera1":
            speakerCameraEstimatedYaw = yaw;
            break;
          default:
            System.out.println("This shouldn't print.");
            break;
        }
        return yaw;
       
    }

    /**
     * Returns pitch angle of the target in the camera space
     * 
     * @param camera camera to use
     * @param targetTagID desired AprilTag
     * @return pitch angle of target in degrees
     */
    public static double getPitch(PhotonCamera camera, int targetTagID) {
        double pitch = 180.0;
        var result = camera.getLatestResult();

        if(result.hasTargets()) {

            var targets = result.getTargets();

            for(int i = 0; i < targets.size(); i++) {

                if(targets.get(i).getFiducialId() == targetTagID) {

                    pitch = targets.get(i).getPitch();

                }

            }

        }
        switch (camera.getName()) {
          case "Camera6":
            ampCameraEstimatedPitch = pitch;
            break;
          case "Camera1":
            speakerCameraEstimatedPitch = pitch;
            break;
          default:
            System.out.println("This shouldn't print.");
            break;
        }
        return pitch;

    }

    /**
     * Returns the area of the camera field of view that the target takes up
     * 
     * @param camera camera to use
     * @param targetTagID desired AprilTag
     * @return area of target as a percent from 0-100
     */
    public static double getArea(PhotonCamera camera, int targetTagID) {

        var result = camera.getLatestResult();

        if(result.hasTargets()) {

            var targets = result.getTargets();

            for(int i = 0; i < targets.size(); i++) {

                if(targets.get(i).getFiducialId() == targetTagID) {

                    return targets.get(i).getArea();

                }

            }

        } else {

            return 0.0;

        }

        return 0.0;

    }


    /**
     * Returns 
     * 
     * @param camera
     * @return
     */
    public static double getTagAmbiguity(PhotonCamera camera, int targetTagID) {
        var result = camera.getLatestResult();

        if(result.hasTargets()) {

            var targets = result.getTargets();

            for(int i = 0; i < targets.size(); i++) {

                if(targets.get(i).getFiducialId() == targetTagID) {

                    return targets.get(i).getPoseAmbiguity();

                }

            }

        } else {

            return 1.0;

        }

        return 1.0;

    }
  /*
   * Logs the estimated pose  and time stamp of the ampCamera
   */
  public static void logAmpCameraPose(Pose3d pose) {
      ampCameraPose = new double[] {
        pose.getX(),
        pose.getY(),
        pose.getZ(),
        pose.getRotation().getAngle(),
    };

  }

  /*
   * Logs the estimated pose and time stamp of the speakerCamera
   */  public static void logSpeakerCameraPose(Pose3d pose) {
    speakerCameraPose = new double[] {
        pose.getX(),
        pose.getY(),
        pose.getZ(),
        pose.getRotation().getAngle(),
    };
  }

  public static double getRobotToRedSpeaker() {
    return speakerPose3d.getTranslation().toTranslation2d().getDistance(redSpeakerTranslation);
  }

  public static double getRobotToBlueSpeaker() {
    return speakerPose3d.getTranslation().toTranslation2d().getDistance(blueSpeakerTranslation);
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Cameras");
    builder.addDoubleProperty("Amp Pitch", () -> ampCameraEstimatedPitch, null);
    builder.addDoubleProperty("Speaker Pitch", () -> speakerCameraEstimatedPitch, null);
    builder.addDoubleProperty("Amp Yaw", () -> ampCameraEstimatedYaw, null);
    builder.addDoubleProperty("Speaker Yaw", () -> speakerCameraEstimatedYaw, null);
    builder.addDoubleProperty("Robot to Red Speaker", Cameras::getRobotToRedSpeaker, null);
    builder.addDoubleProperty("Robot to Blue Speaker", Cameras::getRobotToBlueSpeaker, null);
    builder.addDoubleArrayProperty("Amp Pose", () -> ampCameraPose, null);
    builder.addDoubleArrayProperty("Speaker Pose", () -> speakerCameraPose, null);
  }

}
