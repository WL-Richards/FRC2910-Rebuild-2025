// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

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
}
