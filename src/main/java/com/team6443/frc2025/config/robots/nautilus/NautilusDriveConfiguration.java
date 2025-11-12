// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.config.robots.nautilus;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.team6443.frc2025.config.robots.nautilus.swerve_tunings.NautilusSwerveConstantsComp;
import com.team6443.lib.can.CANDeviceID;
import com.team6443.lib.config.odometry.OdometryStandardDevs;
import com.team6443.lib.config.robot.Pigeon2GyroConfiguration;
import com.team6443.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.team6443.lib.config.subsystems.drive.DrivetrainSimConfiguration;
import com.team6443.lib.config.swerve.SwerveModuleConfiguration;
import com.team6443.lib.config.swerve.TalonFXSwerveModuleConfiguration;
import com.team6443.lib.config.wrappers.ConfigureSlot0Gains;
import com.team6443.lib.mechanics.MultistageGearBox;
import com.team6443.lib.motors.interfaces.MotorIO.NeutralMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/** 
 * Configuration for the nautilius drivetrain
 */
public class NautilusDriveConfiguration {
    // --- Odometry Config ---
    private class Odometry{
        private static final OdometryStandardDevs kDisabledModeStandardDevs = new OdometryStandardDevs(1, 1, 1);

        private static final OdometryStandardDevs kEnabledModeStandardDevs = new OdometryStandardDevs(0.3, 0.3, 0.2);
    }

    public final Pigeon2GyroConfiguration kGyroConfiguration;

    // -------------- Module Configurations --------------
    public final NautilusSwerveModuleConfigurations kSwerveModuleConfigurations;
    
    public final double kChassisTranslationSpeedThreshold = 0.05; // Anything less than this chassis speed in meters per second will be set to 0
    public final double kChassisRotationalSpeedThreshold = 0.05; // Anything less than this chassis speed in radians per second will be set to 0
    public final double kMaxDriveSpeed = 3.6; // Max speed of the robot in meters per second
    public final double kMaxAngularRate = 8.2; // Radians per second

    public final DrivetrainConfiguration kDrivetrainConfiguration;
    public final DrivetrainSimConfiguration kSimulatedDrivetrainConfiguration;

    public NautilusDriveConfiguration(
        NautilusPhysicalConfiguration physicalConfiguration,
        Pigeon2GyroConfiguration gyroConfig,  
        String driveSubsystemName, 
        String driveSubsystemCANBus
    ){
        this.kGyroConfiguration = gyroConfig;

        // Configure swerve modules
        this.kSwerveModuleConfigurations =
        new NautilusSwerveModuleConfigurations(
            driveSubsystemName, 
            driveSubsystemCANBus
        );

        // Configure drive train
        this.kDrivetrainConfiguration = 
        new DrivetrainConfiguration()
                .withName(driveSubsystemName)
                .withMaxDriveSpeed(kMaxDriveSpeed)
                .withMaxAngularRate(kMaxAngularRate)
                .withChassisSpeedDeadband(
                    kChassisTranslationSpeedThreshold, 
                    kChassisRotationalSpeedThreshold
                )
                .withDrivetrainConstants(
                    new SwerveDrivetrainConstants()
                        .withCANBusName(driveSubsystemCANBus)
                        .withPigeon2Id(this.kGyroConfiguration.kCANDevice.getDeviceID())
                        .withPigeon2Configs(this.kGyroConfiguration.kConfiguration)
                )
                // .withDrivetrainConstants(NautilusSwerveConstantsComp.DrivetrainConstants) // Phoenix Tuner Supplied Constants
                .withModuleConstants(
                    new SwerveModuleConstants<?, ?, ?>[]{
                        this.kSwerveModuleConfigurations.kFrontLeftModule.getModuleConstants(),
                        this.kSwerveModuleConfigurations.kFrontRightModule.getModuleConstants(),
                        this.kSwerveModuleConfigurations.kBackLeftModule.getModuleConstants(),
                        this.kSwerveModuleConfigurations.kBackRightModule.getModuleConstants()
                    }
                )
                // .withModuleConstants(NautilusSwerveConstantsComp.kSwerveModuleConstants) // Phoenix Tuner Supplied Constants
                .withOdometryStandardDevs(
                    Odometry.kEnabledModeStandardDevs,
                    Odometry.kDisabledModeStandardDevs
                )
                .withJoystickDeadband(
                    0.05, 
                    0.05
                );

        // Configure simulated drive train
        this.kSimulatedDrivetrainConfiguration = 
        new DrivetrainSimConfiguration(0.005) // 5 ms
                .withName(driveSubsystemName)
                .withRobotWeightPounds(physicalConfiguration.kRobotWeightPounds)
                .withBumperLengthMeters(physicalConfiguration.kBumperLengthM)
                .withBumperWidthMeters(physicalConfiguration.kBumperWidthM)
                .withWheelCoefficientOfFriction(1.2)
                .withDriveMotorCount(1)
                .withSteerMotorCount(1);

    }
}
