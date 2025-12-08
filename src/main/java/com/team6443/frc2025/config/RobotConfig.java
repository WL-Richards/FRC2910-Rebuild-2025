// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.config;

import java.util.List;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6443.frc2025.config.robots.nautilus.Nautilus;
import com.team6443.lib.config.autonomous.ChoreoPatherConfiguration;
import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.config.camera.SimulatedCameraConfiguration;
import com.team6443.lib.config.motors.ServoMotorFollowerConfiguration;
import com.team6443.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.team6443.lib.config.subsystems.drive.simulation.DrivetrainSimConfiguration;
import com.team6443.lib.config.swerve.SwerveModuleConfiguration;
import com.team6443.lib.subsystems.simulation.elevator.SimulatedElevator;

/**
 * Define the basis for what needs to be provided for the robot to be controlled to its fullest.
 * This class should not be used by itself
 * 
 */
public abstract class RobotConfig {

    /**
     * Get the name of the robot has a human readable string
     * @return Robot name as a string
     */
    public abstract String getRobotName();

    /**
     * Configuration that will be passed into the drivetrain configuration
     * 
     * @return The current robot drivetrain configuration
     */
    public abstract DrivetrainConfiguration getDrivetrainConfiguration();

    /**
     * Retrieve the configuration for the swerve modules on the robot
     * @return List of SwerveModuleConfigurations in FL, FR, BL, BR order
     */
    public abstract List<SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>> getSwerveConfigurations();
    
    /**
     * Get configuration details about the drive train sim for this robot
     * @return DrivetrainSimConfiguration of the given robot
     */
    public abstract DrivetrainSimConfiguration getSimulatedDrivetrainConfiguration();

    /**
     * Retrieve the given ChoreoPather module for this robot configuration
     * @return ChoreoPather module for the given robot configuration
     */
    public abstract ChoreoPatherConfiguration getChoreoPatherConfiguration();

    /**
     * Get the configuration for the Elevator subsystem on this bot
     * @return The elevator subsystem configuration
     */
    public abstract ServoMotorFollowerConfiguration<TalonFXConfiguration> getElevatorConfiguration();
    public abstract SimulatedElevator.SimulatedElevatorConfiguration getSimulatedElevatorConfiguration();


    /**
     * Pure abstract method, retrieves how all the cameras on the bot are configured
     * @return Current configuration of sensor and motor ports and IDs
     */
    public abstract List<CameraConfiguration> getCameraConfigurations();

    /**
     * Get the list of simulated camera configurations
     * @return The list of simulated camera configuration
     */
    public abstract List<SimulatedCameraConfiguration> getSimulatedCameraConfigurations();

    /**
     * Pure abstract method, retrieves the list of named CAN buses that are present on this bot
     * @return Current list of CAN buses used on this robot
     */
    public abstract List<String> getCANBusNames();

    /**
     * retrieve the correct robot constants based on the determined robot identification
     * @param identification Robot identification representing what robot is currently in use
     * @return The desired configuration that should be utilized with this robot
     */
    public static RobotConfig getRobotConstants(RobotID identification){
        switch (identification) {
            case NAUTILUS:
                return new Nautilus();
            default:
                return new Nautilus();
        }
    }
}
