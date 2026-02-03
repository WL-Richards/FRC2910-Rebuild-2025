// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.constants;

/**
 * Constants used to compute information about the current robot state
 * 
 * Such constants include, time based odometry buffer memory length, etc.
 */
public final class RobotStateConstants {

    /**
     * All constants related to robot state kinematics should reside within this class
     */
    public class Kinematics {
        
        // Length of time in seconds that the robot pose should be tracked
        public static final double kRobotPoseWindowLengthSeconds = 1.0;
    }

     /**
     * All constants related to robot state vision should reside within this class
     */
    public class Vision {
        
        // Maximum allowed ambiguity for single-tag pose estimates.
        // Ambiguity ranges 0-1; higher = more uncertain which pose solution is correct.
        // Estimates above this threshold are rejected.
        public static final double kDefaultAmbiguityThreshold = 0.19;

        // Maximum allowed yaw difference (degrees) between vision estimate and prior pose.
        // For distant single tags (area < kTagAreaThresholdForYawCheck), if vision yaw
        // disagrees with odometry by more than this, the estimate is rejecte.
        public static final double kDefaultYawDiffThreshold = 5.0;

        // Tag area threshold (% of image) below which yaw sanity check is applied.
        // Small/distant tags have less reliable yaw estimates, so we cross-check
        // against odometry. Above this area, vision yaw is trusted directly.
        public static final double kTagAreaThresholdForYawCheck = 2.0;

        // Minimum tag area (% of image) required to accept single-tag Megatag estimates.
        // Tags smaller than this are too distant for reliable single-tag pose.
        public static final double kTagMinAreaForSingleTagMegatag = 1.0;

        // Maximum allowed |Z| height (meters) of the estimated robot pose.
        // The robot should be on the ground; large Z values indicate a bad solve.
        public static final double kDefaultZThreshold = 0.2;

        // Minimum distance from field origin (meters) for valid pose estimates.
        // Poses near (0,0) are likely failed solves or uninitialized data.
        public static final double kDefaultNormThreshold = 1.0;

        // Time window (seconds) to look back for peak angular velocity.
        // Checks the robot's yaw rate from (timestamp - 0.3s) to timestamp.
        public static final double kHighYawLookbackS = 0.3;

        // Maximum allowed yaw rate (rad/s) for gyro-fused estimates.
        // 5.0 rad/s ≈ 286°/s. If the robot was spinning faster than this
        // during the lookback window, the estimate is rejected.
        public static final double kHighYawVelocityRadS = 3.3;

        // Large variance used to downweight unreliable vision measurements
        public static final double kLargeVariance = 1e6;

    }
}
