// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/** Includes camera setup and all methods for getting camera data */
public class Cameras implements Sendable{

    public static PhotonCamera ampCamera;
    public static PhotonCamera speakerCamera;
    //public static PhotonCamera noteCamera = new PhotonCamera("Camera7");

    private final static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private final static Transform3d robotToAmpCamera = new Transform3d(
        new Translation3d(VisionConstants.AMP_CAMERA_X, VisionConstants.AMP_CAMERA_Y, VisionConstants.AMP_CAMERA_Z),
        new Rotation3d(0, VisionConstants.AMP_CAMERA_PITCH, VisionConstants.AMP_CAMERA_YAW)
    );

    private final static Transform3d robotToSpeakerCamera = new Transform3d(
        new Translation3d(VisionConstants.SPEAKER_CAMERA_X, VisionConstants.SPEAKER_CAMERA_Y, VisionConstants.SPEAKER_CAMERA_Z),
        new Rotation3d(0, VisionConstants.SPEAKER_CAMERA_PITCH, VisionConstants.SPEAKER_CAMERA_YAW)
    );

    private final static PhotonPoseEstimator ampCameraPoseEstimator = new PhotonPoseEstimator(
      aprilTagFieldLayout, 
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      ampCamera, 
      robotToAmpCamera
    );

    private final static PhotonPoseEstimator speakerCameraPoseEstimator = new PhotonPoseEstimator(
      aprilTagFieldLayout, 
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      speakerCamera, 
      robotToSpeakerCamera
    );

    private static double ampCameraEstimatedYaw = 0;
    private static double ampCameraEstimatedPitch = 0;

    private static double speakerCameraEstimatedYaw = 0;
    private static double speakerCameraEstimatedPitch = 0;

    private static double[] ampCameraPose = {0,0,0};
    private static double[] speakerCameraPose = {0,0,0};

    public Cameras() {
      SendableRegistry.add(this, "Cameras");
      ampCamera = new PhotonCamera("Camera6");
      speakerCamera = new PhotonCamera("Camera1");
      Shuffleboard.getTab("Cameras").add(this);
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

  public static void logAmpCameraPose() {
    var estimatedPose = ampCameraPoseEstimator.update();
    if (estimatedPose.isPresent()) {
      var pose = estimatedPose.get().estimatedPose.toPose2d();
      ampCameraPose = new double[] {
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees()
      };
    }

  }

  public static void logSpeakerCameraPose() {
    var estimatedPose = speakerCameraPoseEstimator.update();
    if (estimatedPose.isPresent()) {
      var pose = estimatedPose.get().estimatedPose.toPose2d();
      speakerCameraPose = new double[] {
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees()
      };
    }

  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Cameras");
    builder.addDoubleProperty("Amp Pitch", () -> ampCameraEstimatedPitch, null);
    builder.addDoubleProperty("Speaker Pitch", () -> speakerCameraEstimatedPitch, null);
    builder.addDoubleProperty("Amp Yaw", () -> ampCameraEstimatedYaw, null);
    builder.addDoubleProperty("Speaker Yaw", () -> speakerCameraEstimatedYaw, null);
    builder.addDoubleArrayProperty("Amp Pose", () -> ampCameraPose, null);
    builder.addDoubleArrayProperty("Speaker Pose", () -> speakerCameraPose, null);
  }

}
