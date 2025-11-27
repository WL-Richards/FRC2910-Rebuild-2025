// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.config.robots.nautilus;

import java.util.List;

import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.config.camera.SimulatedCameraConfiguration;
import com.team6443.lib.config.camera.CameraConfiguration.Location;
import com.team6443.lib.config.camera.CameraConfiguration.Resolution;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Configures cameras for the nautilus robot
 */
public class NautilusCameraConfiguration {

    // Target Camera FPS for simulated cameras
    private final double kSimulatedCameraFPS = 120;
    
    // Target Camera latency for simulated cameras
    private final double kSimulatedLatencyMS = 0;

    // Target Camera latency variablity for simulated cameras
    private final double kSimulatedLatencyStdevMS = 0;


    // ------ Front Left Camera ------
    public final CameraConfiguration kFrontLeftCameraConfiguration = new CameraConfiguration(Location.FRONT_LEFT)
        .withCameraPose(
            new Translation3d(
                Units.inchesToMeters(9),            // X (forward)
                Units.inchesToMeters(6),            // Y (right)
                Units.inchesToMeters(8.479)         // Z (up)
            ),
            new Rotation3d(
                Units.degreesToRadians(0),          // Roll
                Units.degreesToRadians(0),          // Pitch
                Units.degreesToRadians(0)           // Yaw
            )
        )
        .withCameraDistanceScalar(1,1)
        .withCameraType(CameraConfiguration.Type.LIMELIGHT)
        .withCameraResolution(Resolution._1280x960);


    public final SimulatedCameraConfiguration kFrontLeftSimulatedCameraConfiguration = new SimulatedCameraConfiguration(kFrontLeftCameraConfiguration)
        .withFramerate(kSimulatedCameraFPS)
        .withCameraNoise(0, 0)
        .withCameraLatency(kSimulatedLatencyMS, kSimulatedLatencyStdevMS)
        .withCameraDistanceScalar(0.940,1.154);
        //1.272, 1.454
    // -------------------------------

    // ------ Front Right Camera ------
    public final CameraConfiguration kFrontRightCameraConfiguration = new CameraConfiguration(Location.FRONT_RIGHT)
        .withCameraPose(
            new Translation3d(
                Units.inchesToMeters(9),    // X (forward)
                Units.inchesToMeters(-6), // Y (right)
                Units.inchesToMeters(8.479)   // Z (up)
            ),
            new Rotation3d(
            Units.degreesToRadians(0),    // Roll
            Units.degreesToRadians(0),            // Pitch
            Units.degreesToRadians(0)       // Yaw
            )
        )
        .withCameraDistanceScalar(1, 1)
        .withCameraType(CameraConfiguration.Type.LIMELIGHT)
        .withCameraResolution(Resolution._1280x960);

    public final SimulatedCameraConfiguration kFrontRightSimulatedCameraConfiguration = new SimulatedCameraConfiguration(kFrontRightCameraConfiguration)
        .withFramerate(kSimulatedCameraFPS)
        .withCameraNoise(0, 0)
        .withCameraLatency(kSimulatedLatencyMS, kSimulatedLatencyStdevMS)
        .withCameraDistanceScalar(0.950,1.154);
    // ---------------------------------

    // ------ Back Left Camera ------
    public final CameraConfiguration kBackLeftCameraConfiguration = new CameraConfiguration(Location.BACK_LEFT)
        .withCameraPose(
            new Translation3d(
                Units.inchesToMeters(-11),              // X (forward)
                Units.inchesToMeters(6.75),            // Y (right)
                Units.inchesToMeters(8.440)          // Z (up)
            ),
            new Rotation3d(
                Units.degreesToRadians(0),      // Roll
                Units.degreesToRadians(0),          // Pitch
                Units.degreesToRadians(180)     // Yaw
            )
        )
        .withCameraDistanceScalar(1, 1)
        .withCameraType(CameraConfiguration.Type.LIMELIGHT)
        .withCameraResolution(Resolution._1280x960);


    public final SimulatedCameraConfiguration kBackLeftSimulatedCameraConfiguration = new SimulatedCameraConfiguration(kBackLeftCameraConfiguration)
        .withFramerate(kSimulatedCameraFPS)
        .withCameraNoise(0, 0)
        .withCameraLatency(kSimulatedLatencyMS, kSimulatedLatencyStdevMS)
        .withCameraDistanceScalar(0.900,1.090);
    // -----------------------------

    // ------ Back Right Camera  ------
    public final CameraConfiguration kBackRightCameraConfiguration = new CameraConfiguration(Location.BACK_RIGHT)
        .withCameraPose(
            new Translation3d(
                Units.inchesToMeters(-11),              // X (forward)
                Units.inchesToMeters(-6.75),            // Y (right)
                Units.inchesToMeters(8.440)          // Z (up)
            ),
            new Rotation3d(
                Units.degreesToRadians(0),      // Roll
                Units.degreesToRadians(0),          // Pitch
                Units.degreesToRadians(180)     // Yaw
            )
        )
        .withCameraDistanceScalar(1, 1) 
        .withCameraType(CameraConfiguration.Type.LIMELIGHT)
        .withCameraResolution(Resolution._1280x960);


    public final SimulatedCameraConfiguration kBackRightSimulatedCameraConfiguration = new SimulatedCameraConfiguration(kBackRightCameraConfiguration)
        .withFramerate(kSimulatedCameraFPS)
        .withCameraNoise(0, 0)
        .withCameraLatency(kSimulatedLatencyMS, kSimulatedLatencyStdevMS)
        .withCameraDistanceScalar(0.890,1.090);
        
    // --------------------------------

    // List of configurations in FL, FR, BL, BR order
    public final List<CameraConfiguration> kCameraConfigurations = List.of(
        kFrontLeftCameraConfiguration,
        kFrontRightCameraConfiguration,
        kBackLeftCameraConfiguration,
        kBackRightCameraConfiguration
    );

    // List of configurations in FL, FR, BL, BR order
    public final List<SimulatedCameraConfiguration> kSimulatedCameraConfigurations = List.of(
        kFrontLeftSimulatedCameraConfiguration,
        kFrontRightSimulatedCameraConfiguration,
        kBackLeftSimulatedCameraConfiguration,
        kBackRightSimulatedCameraConfiguration
    );

    public NautilusCameraConfiguration(){}

}
