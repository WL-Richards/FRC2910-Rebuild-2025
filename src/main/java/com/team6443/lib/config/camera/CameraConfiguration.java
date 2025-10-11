// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.camera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Properties representing the configuration of a camera for a given robot
 */
public final class CameraConfiguration {

    /**
     * Camera location enum that stores both a user describes camera location as well as its true Pose3d, note Rotation3d uses RADIANS
     * 
     * 
     * NOTE: All relative directions (right and left) are from the perspective of looking from the rear* of the robot to the front*
     * *these must be agreed upon, although usually it is clear
     */
    public enum Location {
        FRONT_RIGHT("Front Right"),
        FRONT_LEFT("Front Left"),
        BACK_LEFT("Back Left"),
        BACK_RIGHT("Back Right"),
        UNCONFIGURED("Unconfigured"),
        ;

        // --- End of Enum Definitions ---

        /* Human readable name for the camera location */
        public final String LocationName;

        /* Current pose of the camera relative to the center of the robot, i.e robot center = (0,0,0) */
        public Pose3d CameraPose;

        private Location(String name)
        {
            this.LocationName = name;
        }

        /**
         * Retrieve the name of the location as a string
         * @return Stringified version of the camera location name
         */
        @Override
        public String toString(){
            return LocationName;
        }

        /**
         * Update the current pose of a given camera location relative to the center of the robot
         * @param pose The new Pose3d that we wish to have the camera located at
         */
        public void withCameraPose(Pose3d pose){
            this.CameraPose = pose;
        }

    }

    /**
     * Enum + user described camera type
     */
    public enum Type {
        LIMELIGHT("Limelight"),
        UNCONFIGURED("Unconfigured"),
        ;

        /* Human readable name for the camera location */
        public final String Name;

        private Type(String name)
        {
            this.Name = name;
        }

        /**
         * Get the name 
         * @return
         */
        @Override
        public String toString(){
            return this.Name;
        }
    }

    // Location of this camera
    public Location CameraLocation = Location.UNCONFIGURED;

    // Type of the camera
    public Type CameraType = Type.UNCONFIGURED;

    // What is the ratio between the reported distance to a tag and the actual distance to a tag, defaults to 1
    public double CameraDistanceScalar = 1.0;

    /**
     * Pass in a location when the camera is configured
     * @param location Location of the camera on the robot
     */
    public CameraConfiguration(Location location){
        this.CameraLocation = location;
    }

    /**
     * Update the pose of this camera relative to the center of the robot
     * @param cameraOffset Camera offset translation relative to the center of the robot (z or height is the distance FROM THE FLOOR), in meters
     * @param cameraRotation The rotation of the camera in world to itself, in radians
     * @return Reference to this camera configuration object
     */
    public CameraConfiguration withCameraPose(Translation3d cameraOffset, Rotation3d cameraRotation){
        this.CameraLocation.withCameraPose(new Pose3d(cameraOffset, cameraRotation));
        return this;
    }

    /**
     * Update type of camera that is used here
     * @param type What type of camera is in use in this configuration
     * @return Reference to this camera configuration object
     */
    public CameraConfiguration withCameraType(Type type){
        this.CameraType = type;
        return this;
    }

    /**
     * Configure the scalar/ratio that needs to be provided to account for distance irregularities in vision
     * Units don't matter but both actual and reported must use same, I suggest making all measurements in inches
     * 
     * @param actual    The actual distance to some landmark (april tag)
     * @param reported  The distance reported by vision to some landmark (april tag)
     * @return Reference to this camera configuration object
     */
    public CameraConfiguration withCameraDistanceScalar(double actual, double reported){
        this.CameraDistanceScalar = actual / reported;
        return this;
    }

}
