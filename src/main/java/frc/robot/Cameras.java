// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

/** Includes camera setup and all methods for getting camera data */
public class Cameras {

    public static PhotonCamera ampCamera = new PhotonCamera("Camera6");
    public static PhotonCamera speakerCamera = new PhotonCamera("Camera1");
    //public static PhotonCamera noteCamera = new PhotonCamera("Camera7");


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

        var result = camera.getLatestResult();

        if(result.hasTargets()) {

            var targets = result.getTargets();

            for(int i = 0; i < targets.size(); i++) {

                if(targets.get(i).getFiducialId() == targetTagID) {

                    return targets.get(i).getYaw();

                }

            }

        } else {

            return 180.0;

        }

        return 180.0;
       
    }

    /**
     * Returns pitch angle of the target in the camera space
     * 
     * @param camera camera to use
     * @param targetTagID desired AprilTag
     * @return pitch angle of target in degrees
     */
    public static double getPitch(PhotonCamera camera, int targetTagID) {
        
        var result = camera.getLatestResult();

        if(result.hasTargets()) {

            var targets = result.getTargets();

            for(int i = 0; i < targets.size(); i++) {

                if(targets.get(i).getFiducialId() == targetTagID) {

                    return targets.get(i).getPitch();

                }

            }

        } else {

            return 180.0;

        }

        return 180.0;

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

}
