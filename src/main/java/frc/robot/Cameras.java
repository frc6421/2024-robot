// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;

/** Includes camera setup and all methods for getting camera data */
public class Cameras {

    //TODO update camera names
    public static PhotonCamera ampCamera = new PhotonCamera("Camera6");
    public static PhotonCamera speakerCamera = new PhotonCamera("Camera1");
    public static PhotonCamera noteCamera = new PhotonCamera("Camera7");


    public static boolean isTarget(PhotonCamera camera) {
        return camera.getLatestResult().hasTargets();
    }

    /**
     * Returns yaw angle of the target in the camera space
     * 
     * @param camera camera to use
     * @param targetTagID desired AprilTag
     * @return yaw angle of target in radians
     */
    public static double getYaw(PhotonCamera camera, int targetTagID) {

        var result = camera.getLatestResult();

        if(result.hasTargets()) {

            var targets = result.getTargets();

            for(int i = 0; i < targets.size(); i++) {

                if(targets.get(i).getFiducialId() == targetTagID) {

                    return Units.degreesToRadians(targets.get(i).getYaw());

                }

            }

        } else {

            return 0.0;

        }

        return 0.0;
       
    }

    /**
     * Returns pitch angle of the target in the camera space
     * 
     * @param camera camera to use
     * @param targetTagID desired AprilTag
     * @return pitch angle of target in radians
     */
    public static double getPitch(PhotonCamera camera, int targetTagID) {
        
        var result = camera.getLatestResult();

        if(result.hasTargets()) {

            var targets = result.getTargets();

            for(int i = 0; i < targets.size(); i++) {

                if(targets.get(i).getFiducialId() == targetTagID) {

                    return Units.degreesToRadians(targets.get(i).getPitch());

                }

            }

        } else {

            return 0.0;

        }

        return 0.0;

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

            return 0.0;

        }

        return 0.0;

    }

}
