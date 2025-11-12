// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.config.robots.nautilus;

import java.util.List;

import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.config.camera.CameraConfiguration.Location;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Configures cameras for the nautilus robot
 */
public class NautilusCameraConfiguration {

    // ------ Front Left Camera ------
    public final CameraConfiguration kFrontLeftCameraConfiguration = new CameraConfiguration(Location.FRONT_LEFT)
        .withCameraPose(
            new Translation3d(
                Units.inchesToMeters(9),         // X (forward)
                Units.inchesToMeters(8.25),      // Y (right)
                Units.inchesToMeters(7.5)        // Z (up)
            ),
            new Rotation3d(
                Units.degreesToRadians(180),    // Roll
                Units.degreesToRadians(-15),            // Pitch
                Units.degreesToRadians(0)       // Yaw
            )
        )
        .withCameraDistanceScalar(24.25, 24.06) // TODO: DETERMINE
        .withCameraType(CameraConfiguration.Type.LIMELIGHT);
    // -------------------------------

    // ------ Front Right Camera ------
    public final CameraConfiguration kFrontRightCameraConfiguration = new CameraConfiguration(Location.FRONT_RIGHT)
        .withCameraPose(
            new Translation3d(
                Units.inchesToMeters(9),    // X (forward)
                Units.inchesToMeters(-8.25),       // Y (right)
                Units.inchesToMeters(7.5)   // Z (up)
            ),
            new Rotation3d(
            Units.degreesToRadians(180),    // Roll
            Units.degreesToRadians(-15),            // Pitch
            Units.degreesToRadians(0)       // Yaw
            )
        )
        .withCameraDistanceScalar(24.25, 24.24) // TODO: DETERMINE
        .withCameraType(CameraConfiguration.Type.LIMELIGHT);
    // ---------------------------------

    // ------ Back Left Camera ------
    public final CameraConfiguration kBackLeftCameraConfiguration = new CameraConfiguration(Location.BACK_LEFT)
        .withCameraPose(
            new Translation3d(
                Units.inchesToMeters(-11),              // X (forward)
                Units.inchesToMeters(11.5),      // Y (right)
                Units.inchesToMeters(6)          // Z (up)
            ),
            new Rotation3d(
                Units.degreesToRadians(0),      // Roll
                Units.degreesToRadians(-23.5),          // Pitch
                Units.degreesToRadians(147)     // Yaw
            )
        )
        .withCameraDistanceScalar(10.25, 10.77) // TODO: DETERMINE
        .withCameraType(CameraConfiguration.Type.LIMELIGHT);
    // -----------------------------

    // ------ Back Right Camera  ------
    public final CameraConfiguration kBackRightCameraConfiguration = new CameraConfiguration(Location.BACK_RIGHT)
        .withCameraPose(
            new Translation3d(
                Units.inchesToMeters(-11),              // X (forward)
                Units.inchesToMeters(-11.5),            // Y (right)
                Units.inchesToMeters(6)          // Z (up)
            ),
            new Rotation3d(
                Units.degreesToRadians(0),      // Roll
                Units.degreesToRadians(-23.5),          // Pitch
                Units.degreesToRadians(147)     // Yaw
            )
        )
        .withCameraDistanceScalar(10.25, 10.88) // TODO: DETERMINE
        .withCameraType(CameraConfiguration.Type.LIMELIGHT);
    // --------------------------------

    // List of configurations in FL, FR, BL, BR order
    public final List<CameraConfiguration> kCameraConfigurations = List.of(
        kFrontLeftCameraConfiguration,
        kFrontRightCameraConfiguration,
        kBackLeftCameraConfiguration,
        kBackRightCameraConfiguration
    );

    public NautilusCameraConfiguration(){}

}
