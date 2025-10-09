// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config.base;

import java.util.List;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import frc.robot.config.Spectre;
import frc.robot.util.config.NamedCANCoderConfiguration;
import frc.robot.util.config.NamedTalonFXConfiguration;

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
     * Pure abstract method, retrieves the phoenix 6 swerve drive constants to be used by this robot configuration
     * @return Constants utilized by the swerve drive
     */
    public abstract SwerveDrivetrainConstants getSwerveDriveConstants();

    /**
     * Pure abstract method, retrieves the phoenix 6 swerve drive constants for each module utilized in the swerve drive
     * @return Constants utilized by each swerve drive module
     */
    public abstract List<SwerveModuleConstants<NamedTalonFXConfiguration, NamedTalonFXConfiguration, NamedCANCoderConfiguration>> getModuleConstants();

    /**
     * Pure abstract method, retrieves how all motors and sensors are mapped to their respective CAN IDs and RIO ports
     * @return Current configuration of sensor and motor ports and IDs
     */
    public abstract PortConfiguration getPortConfiguration();

    /**
     * Pure abstract method, retrieves how all the cameras on the bot are configured
     * @return Current configuration of sensor and motor ports and IDs
     */
    public abstract List<CameraConfiguration> getCameraConfigurations();

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
            case SPECTRE:
                return new Spectre();
            default:
                return new Spectre();
        }
    }
}
