// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.config.robots.nautilus;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6443.frc2025.config.robots.nautilus.swerve_tunings.NautilusSwerveConstantsComp;
import com.team6443.lib.can.CANDeviceID;
import com.team6443.lib.config.swerve.SwerveModuleConfiguration;
import com.team6443.lib.config.swerve.TalonFXSwerveModuleConfiguration;

/** Configurations for the nautilus robot swerve modules */
public class NautilusSwerveModuleConfigurations {
    // ---------------------- Front Left -----------------------
    public final SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kFrontLeftModule;

    // ---------------------- Front Right ----------------------
    public final SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kFrontRightModule;

    // ---------------------- Back Left ------------------------
    public final SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kBackLeftModule;

    // ---------------------- Back Right -----------------------
    public final SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kBackRightModule;

    public NautilusSwerveModuleConfigurations(String driveSubsystemName, String driveCANBusName){

        // --- Front Left Module ---
        this.kFrontLeftModule = new TalonFXSwerveModuleConfiguration(
            NautilusSwerveConstantsComp.FrontLeft,
            "FrontLeftSwerveModule", 

            // Drive Motor
            new CANDeviceID(
                5, 
                "FrontRightSwerveDriveMotor", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                driveCANBusName
            ), 

            // Steer Motor
            new CANDeviceID(
            8, 
                "FrontLeftSwerveSteerMotor", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                driveCANBusName
            ), 

            // Steer Encoder
            new CANDeviceID(
            26, 
                "FrontLeftSwerveSteerEncoder", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER, 
                driveCANBusName
            )
        );

        // --- Front Right Module ---
        this.kFrontRightModule = new TalonFXSwerveModuleConfiguration(
            NautilusSwerveConstantsComp.FrontRight,
            "FrontRightSwerveModule", 

            // Drive Motor
            new CANDeviceID(
                5, 
                "FrontRightSwerveDriveMotor", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                driveCANBusName
            ),

            // Steer Motor
            new CANDeviceID(
            6, 
                "FrontRightSwerveSteerMotor", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                driveCANBusName
            ),

            // Steer Encoder
            new CANDeviceID(
            24, 
                "FrontRightSwerveSteerEncoder", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER, 
                driveCANBusName
            )
        );

        // --- Back Left Module ---
        this.kBackLeftModule = new TalonFXSwerveModuleConfiguration(
            NautilusSwerveConstantsComp.BackLeft,
            "BackLeftSwerveModule", 

            // Drive Motor
            new CANDeviceID(
                3, 
                "BackLeftSwerveDriveMotor", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                driveCANBusName
            ),
            
            // Steer Motor
            new CANDeviceID(
            4, 
                "BackLeftSwerveSteerMotor", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                driveCANBusName
            ),

            // Steer Encoder
            new CANDeviceID(
            25, 
                "BackLeftSwerveSteerEncoder", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER, 
                driveCANBusName
            )
        );

        // --- Back Right Module ---
        this.kBackRightModule = new TalonFXSwerveModuleConfiguration(
            NautilusSwerveConstantsComp.BackRight,
            "BackRightSwerveModule", 

            // Drive Motor
            new CANDeviceID(
                9, 
                "BackRightSwerveDriveMotor", 
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                driveCANBusName
            ),

            // Steer Motor
            new CANDeviceID(
                2, 
                "BackRightSwerveSteerMotor",  
                driveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                driveCANBusName
            ),

            // Steer Encoder
            new CANDeviceID(
                23, 
                "BackRightSwerveSteerEncoder",  
                driveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER, 
                driveCANBusName
            )
        );
    }
}
