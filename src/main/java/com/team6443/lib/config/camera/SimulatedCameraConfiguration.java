// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.camera;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Properties representing the configuration of a camera for a given robot
 */
public  class SimulatedCameraConfiguration  {

    // Properties of the simulated camera
    public final SimCameraProperties kSimCameraProperties = new SimCameraProperties();

    // Camera configuration of this camera that is being simulated
    public final CameraConfiguration kCameraConfiguration;

    /**
     * Pass in a location when the camera is configured
     * @param location Location of the camera on the robot
     */
    public SimulatedCameraConfiguration(CameraConfiguration cameraConfig){
        this.kCameraConfiguration = cameraConfig;

        //// --- Derive Diagonal Fov ---
        double hRad = Math.toRadians(this.kCameraConfiguration.CameraFOV.HorizontalDegrees / 2);
        double vRad = Math.toRadians(this.kCameraConfiguration.CameraFOV.VerticalDegrees / 2);

        double diagRad = 2 * Math.atan(Math.sqrt(
                Math.tan(hRad) * Math.tan(hRad) +
                Math.tan(vRad) * Math.tan(vRad)
        ));

        Rotation2d diagFov = new Rotation2d(diagRad);
        //// --------------------------

        withSpecs(
            this.kCameraConfiguration.CameraResolution.XPixels, 
            this.kCameraConfiguration.CameraResolution.YPixels, 
            diagFov
        );

    }

    /**
     * Configure the simulated camera to have certain specifications
     * @param xPixels Number of pixels along x
     * @param yPixels Number of pixels along y
     * @param diagFOV Diagonal FOV 
     * @return this simulated camera configuration for chaining
     */
    public SimulatedCameraConfiguration withSpecs(int xPixels, int yPixels, Rotation2d diagFOV){
        kSimCameraProperties.setCalibration(xPixels, yPixels, diagFOV);
        return this;
    }

    /**
     * Configure the simulated camera to have noise in pixel amounts
     * @param xPixelsNoise How much error in pixels should we add into the simulation along the x
     * @param yPixelsNoise How much error in pixels should we add into the simulation along the y
     * @return this simulated camera configuration for chaining
     */
    public SimulatedCameraConfiguration withCameraNoise(double xPixelNoise, double yPixelNoise){
        kSimCameraProperties.setCalibError(xPixelNoise, yPixelNoise);
        return this;
    }

    /**
     * Configure the simulated camera to have a set framerate
     * @param fps Camera frames per second
     * @return this simulated camera configuration for chaining
     */
    public SimulatedCameraConfiguration withFramerate(double fps){
        kSimCameraProperties.setFPS(fps);
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
    public SimulatedCameraConfiguration withCameraDistanceScalar(double actual, double reported){
        if(RobotBase.isReal()) return this;
        kCameraConfiguration.withCameraDistanceScalar(actual, reported);
        return this;
    }

     /**
     * Configure the simulated camera to have a set framerate
     * @param latencyMS Latency of the camera in milliseconds
     * @param latencyStdDevMS Standard deviation in latency in milliseconds
     * @return this simulated camera configuration for chaining
     */
    public SimulatedCameraConfiguration withCameraLatency(double latencyMS, double latencyStdDevMS){
        kSimCameraProperties.setAvgLatencyMs(latencyMS);
        kSimCameraProperties.setLatencyStdDevMs(latencyStdDevMS);
        return this;
    }

    @Override
    public String toString() {
        return kCameraConfiguration.toString();
    }
}
