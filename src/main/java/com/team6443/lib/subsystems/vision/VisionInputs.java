// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.vision;

import java.util.List;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.team6443.lib.config.camera.CameraConfiguration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** 
 * Represent the state of the vision system
 */
public class VisionInputs implements LoggableInputs{

    /**
     * Represents a valid april tag observation
     */
    public static class AprilTagObservations {
        public final String cameraName;
        public final CameraConfiguration.Location cameraLocation;
        public final int tagID;
        public final Pose2d robotPoseFromCamera;
    
        public AprilTagObservations(
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

    /// Do we have a tag currently detected
    public boolean hasTag = false;                                      
   
    /// Horizontal rotation 2D from our current camera's center to the detected tag
    public Rotation2d horizontalRotationToTag = new Rotation2d();       
    
    /// The ID of the currently detected tag
    public int tagID = -1;           
    
    /// The Corner position
    public List<AprilTagCornerPosition> tagCornerPositions;      
    
    /// The height of the currently detected tag in pixels
    public double tagHeightPixels;                                     
    //                                                                                                                      /|
    /// The Height OF (not to) the currently detected tag as a vertical rotation 2D (the angle that this tag takes up like /_|, say the tag are the '|' then the angle is the angle in the bottom left)
    public Rotation2d tagHeightRotations;

    /// The distance to the tag from our camera in meters
    public double tagDistanceMeters;

    /// The pose of our robot based off the tags location and our distance to the tag WITH compensation for camera latency
    public Pose2d robotPoseBasedOffTagLocationLatencyCompensated;

    /// The pose of our robot based off the tags location and our distance to the tag WITH OUT compensation for camera latency
    public Pose2d robotPoseBasedOffTagLocationLatencyUncompensated;
    
    @Override
    public void toLog(LogTable table) {
        table.put("HasTag", hasTag);
        table.put("HorizontalRotationToTag", horizontalRotationToTag);
        table.put("TagID", tagID);
        table.put("TagHeightPixels", tagHeightPixels);
        table.put("TagHeightRotations", tagHeightRotations);
        table.put("TagDistanceMeters", tagDistanceMeters); 
        table.put("RobotPoseEstimationLatencyCompensated", robotPoseBasedOffTagLocationLatencyCompensated); 
        table.put("RobotPoseEstimationLatencyUncompensated", robotPoseBasedOffTagLocationLatencyUncompensated); 
    }

    @Override
    public void fromLog(LogTable table) {
        hasTag = table.get("HasTag", hasTag);
        horizontalRotationToTag = table.get("HorizontalRotationToTag", horizontalRotationToTag);
        tagID = table.get("TagID", tagID);
        tagHeightPixels = table.get("TagHeightPixels", tagHeightPixels);
        tagHeightRotations = table.get("TagHeightRotations", tagHeightRotations);
        tagDistanceMeters = table.get("TagDistanceMeters", tagDistanceMeters);
        robotPoseBasedOffTagLocationLatencyCompensated = table.get("RobotPoseEstimationLatencyCompensated", robotPoseBasedOffTagLocationLatencyCompensated);
        robotPoseBasedOffTagLocationLatencyUncompensated = table.get("RobotPoseEstimationLatencyUncompensated", robotPoseBasedOffTagLocationLatencyUncompensated);
    }
}
