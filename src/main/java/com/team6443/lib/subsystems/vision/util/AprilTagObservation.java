package com.team6443.lib.subsystems.vision.util;

import com.team6443.lib.config.camera.CameraConfiguration;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Represents an observation of an april tag
 */
public class AprilTagObservation {
    public final String cameraName;
    public final CameraConfiguration.Location cameraLocation;
    public final int tagID;
    public final Pose2d robotPoseFromCamera;

    public AprilTagObservation(
            String cameraName,
            CameraConfiguration.Location cameraLocation,
            int tagID,
            Pose2d robotPoseFromCamera) {

        this.cameraName = cameraName;
        this.cameraLocation = cameraLocation;
        this.tagID = tagID;
        this.robotPoseFromCamera = robotPoseFromCamera;
    }
} 
