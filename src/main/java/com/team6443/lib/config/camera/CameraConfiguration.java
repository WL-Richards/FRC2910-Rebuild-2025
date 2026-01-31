// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.camera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Properties representing the configuration of a camera for a given robot
 */
public class CameraConfiguration {

    /**
     * Camera location enum that stores both a user describes camera location as well as its true Pose3d, note Rotation3d uses RADIANS
     * 
     * 
     * NOTE: All relative directions (right and left) are from the perspective of looking from the rear* of the robot to the front*
     * *these must be agreed upon, although usually it is clear
     */
    public enum Location {
        FRONT_RIGHT("FrontRight"),
        FRONT_LEFT("FrontLeft"),
        BACK_LEFT("BackLeft"),
        BACK_RIGHT("BackRight"),
        UNCONFIGURED("Unconfigured"),
        ;

        // --- End of Enum Definitions ---

        /* Human readable name for the camera location */
        public final String LocationName;

        /* Current pose of the camera relative to the center of the robot, i.e robot center = (0,0,0) */
        public Pose3d CameraPose;

        /** Translation representing the position of the current camera as an offset from the center of the robot */
        public Translation2d TranslationToRobotCenter;

        /** Rotation2d representing the mounting yaw rotation of the camera on the robot relative to its "forward axis" */
        public Rotation2d MountingYaw;

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
        public Location withCameraPose(Pose3d pose){
            this.CameraPose = pose;

            MountingYaw = Rotation2d.fromRadians(this.CameraPose.getRotation().getZ());
            TranslationToRobotCenter = new Translation2d(pose.getX(), pose.getY());
            return this;
        }

        private static Rotation2d extractYaw(Rotation3d rot){
            // Forward direction in camera frame is (0,0,1).
            // Rotate it into robot/world frame:
            Translation3d forward = new Translation3d(0, 0, 1).rotateBy(rot);

            // Project forward vector into X-Y plane
            double x = forward.getX();
            double y = forward.getY();

            // Compute yaw using standard atan2
            return new Rotation2d(Math.atan2(y, x));
        }



    }

    /**
     * Enum + user described camera type
     */
    public enum Type {
        LIMELIGHT("limelight"),
        UNCONFIGURED("Unconfigured"),
        ;

        /* Human readable name for the camera type additionally coresponds to network table name for limelight */
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

    /**
     * Represents the resolution of the camera as a basic struct
     */
    public enum Resolution {
        _1280x960(1280, 960),
        _1280x720(1280, 720),
        _640x480(640, 480),
        ;

        public int XPixels;
        public int YPixels;

        private Resolution(int xPixels, int yPixels){
            this.XPixels = xPixels;
            this.YPixels = yPixels;
        }
     
    }

    /**
     * Enum used to represent common camera FOVs
     */
    public enum FOV {
        LIMELIGHT4(82, 56.2)
        ;

        public double HorizontalDegrees;
        public double VerticalDegrees;

        private FOV(double horizontalDegrees, double verticalDegrees){
            this.HorizontalDegrees = horizontalDegrees;
            this.VerticalDegrees = verticalDegrees;
        }
       
    }

    // Location of this camera
    public Location CameraLocation = Location.UNCONFIGURED;

    // Type of the camera
    public Type CameraType = Type.UNCONFIGURED;

    // Resolution of the camera
    public Resolution CameraResolution = Resolution._1280x960;

    // FOV of the camera
    public FOV CameraFOV = FOV.LIMELIGHT4;

    // What is the ratio between the reported distance to a tag and the actual distance to a tag, defaults to 1
    public double CameraDistanceScalar = 1.0;

    /**
     * 254 Comment:
     *  "When limelight tx was compared to robot rotation from the gyro, it was observed that they did not scale at the same rate. 
     *  Assuming the Pigeon2 scales correctly, this means the limelight's angle scaling is incorrect. 
     *  Because the difference scaled linearly, we found out the ratio between the limelight angle and robot angle and applied it to the tx angle.""
     */
    public double CameraXRotationScalar = 1.0;

    /**
     * Name of the network table that this camera is uploading data to (typically a limelight)
     */
    public String NetworkTableName = null;

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
     * Update resolution of camera
     * @param resolution What resolution of the camera are we using
     * @return Reference to this camera configuration object
     */
    public CameraConfiguration withCameraResolution(Resolution resolution){
        this.CameraResolution = resolution;
        return this;
    }

    /**
     * Update FOV of camera
     * @param fov What FOV the camera is 
     * @return Reference to this camera configuration object
     */
    public CameraConfiguration withCameraFOV(FOV fov){
        this.CameraFOV = fov;
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

    /**
     * Configure the scalar/ratio that needs to be provided to account for X rotation rate irregularities between vision and gyro scope
     * Units don't matter but both actual and reported must use same, I suggest making all measurements in degrees
     * 
     * To determine rotate 
     * 
     * @param gryoChange    The degrees reported by the gyro for some angle
     * @param cameraChange  The degrees reported by the camera for the same angle
     * @return Reference to this camera configuration object
     */
    public CameraConfiguration withXRotationScalar(double gryoChange, double cameraChange){
        this.CameraXRotationScalar = gryoChange / cameraChange;
        return this;
    }


    /**
     * Configure the name of the network table to point to where this camera is uploading its data
     * @param name Name of the network table where data is being uploaded to
     * @return Reference to this camera configuration object
     */
    public CameraConfiguration withNetworkTableName(String name){
        this.NetworkTableName = name;
        return this;
    }

    @Override
    public String toString() {
        return CameraLocation.toString() + "_" + CameraType.toString();
    }

}
