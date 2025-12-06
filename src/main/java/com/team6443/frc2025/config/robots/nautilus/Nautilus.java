// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.config.robots.nautilus;

import java.util.List;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.team6443.frc2025.config.RobotConfig;
import com.team6443.frc2025.config.robots.nautilus.swerve_tunings.NautilusSwerveConstantsComp;
import com.team6443.lib.can.CANDeviceID;
import com.team6443.lib.config.autonomous.ChoreoPathingConfiguration;
import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.config.camera.SimulatedCameraConfiguration;
import com.team6443.lib.config.camera.CameraConfiguration.Location;
import com.team6443.lib.config.motors.ServoMotorFollowerConfiguration;
import com.team6443.lib.config.motors.factories.TalonFXConfigurationFactory;
import com.team6443.lib.config.odometry.OdometryStandardDevs;
import com.team6443.lib.config.robot.PhysicalConfiguration;
import com.team6443.lib.config.robot.Pigeon2GyroConfiguration;
import com.team6443.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.team6443.lib.config.subsystems.drive.simulation.DrivetrainSimConfiguration;
import com.team6443.lib.config.swerve.SwerveModuleConfiguration;
import com.team6443.lib.config.swerve.TalonFXSwerveModuleConfiguration;
import com.team6443.lib.config.wrappers.ConfigureSlot0Gains;
import com.team6443.lib.mechanics.MultistageGearBox;
import com.team6443.lib.motors.interfaces.MotorIO.FollowDirection;
import com.team6443.lib.motors.interfaces.MotorIO.NeutralMode;
import com.team6443.lib.subsystems.simulation.elevator.SimulatedElevator.SimulatedElevatorConfiguration;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * This code represents the configuration for 6443's 2025 robot "Nautilus" 
 */
public class Nautilus extends RobotConfig {

    // --- Misc. Config ---
    private static final String kRobotName = "Nautilus";

    private static final String kElevatorSubsystemName = "ElevatorSubsystem";
    private static final String kDriveSubsystemName = "DriveSubsystem";

    // ---  CAN Bus Config  --- 
    // (Note IDs can be the same for different device types (TalonFX, Pigeon2, etc), ie gyro and front left drive motor have the same ID)
    private static final String kCanivoreBusName = "CANivore";

    // List of can buses that are present on this robot
    private static final List<String> kCANBuses = List.of(
        kCanivoreBusName
    );

    // --- Robot Physical Properties ---
    private static final PhysicalConfiguration kPhysicalConfiguration = 
        new PhysicalConfiguration()
            .withRobotWeightPounds(150)
            .withWheelBaseLengthM(Units.inchesToMeters(22.75))
            .withWheelTrackWidthM(Units.inchesToMeters(22.75))
            .withBumperLengthM(Units.inchesToMeters(35.625))
            .withBumperWidthM(Units.inchesToMeters(35.625))
            .withWheelCoefficientOfFriction(1.2);

    // --- Drivetrain Config ---
    // --- Gyro ---
    private static final Pigeon2GyroConfiguration kGyroConfiguration = 
        new Pigeon2GyroConfiguration()
            .withGyroYawError(0.0)
            .withGyroMountRotation(
                new Rotation3d(
                    Math.toRadians(0),      // Roll
                    Math.toRadians(0),      // Pitch
                    Math.toRadians(0)       // Yaw
                )
            )
            .withCANDevice(
                new CANDeviceID(
                    1, 
                    "DriveTrainGyro",
                    kDriveSubsystemName,
                    CANDeviceID.CANDeviceType.PIGEON2,
                    kCanivoreBusName
                )
            );


    private static final NautilusDriveConfiguration kDriveConfiguration = 
        new NautilusDriveConfiguration(
            kPhysicalConfiguration,
            kGyroConfiguration,
            kDriveSubsystemName, 
            kCanivoreBusName
        );
    
    // --- Camera Config ---
    private static final NautilusCameraConfiguration kCameraConfiguration = new NautilusCameraConfiguration();
    
    // --- Elevator Config ---
    private static final NautilusElevatorConfiguration kElevatorConfiguration = 
        new NautilusElevatorConfiguration(
            kElevatorSubsystemName, 
            kCanivoreBusName
        );

    // --- Autonomous Config ---
    private static final NautilusAutonomousConfiguration kAutonomousConfiguration = new NautilusAutonomousConfiguration();

    

    // --- RobotConfig override functions ---
    @Override
    public String getRobotName() {
        return kRobotName;
    }

    @Override
    public List<CameraConfiguration> getCameraConfigurations() {
        return kCameraConfiguration.kCameraConfigurations;
    }

    @Override
    public List<SimulatedCameraConfiguration> getSimulatedCameraConfigurations() {
        return kCameraConfiguration.kSimulatedCameraConfigurations;
    }

    @Override
    public List<String> getCANBusNames() {
        return kCANBuses;
    }
 
    @Override
    public ChoreoPathingConfiguration getChoreoPathingConfiguration() {
        return kAutonomousConfiguration.kChoreoPathingConfiguration;
    }
    
    @Override
    public DrivetrainConfiguration getDrivetrainConfiguration() {
        return kDriveConfiguration.kDrivetrainConfiguration;
    }

    @Override
    public List<SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>> getSwerveConfigurations() {
        return kDriveConfiguration.kSwerveModuleConfigurations.kConfigurations;
    }

    @Override
    public DrivetrainSimConfiguration getSimulatedDrivetrainConfiguration() {
        return kDriveConfiguration.kSimulatedDrivetrainConfiguration;
    }

    @Override
    public ServoMotorFollowerConfiguration<TalonFXConfiguration> getElevatorConfiguration() {
        return kElevatorConfiguration.kElevatorMotorSubsystemConfiguration;
    }

    @Override
    public SimulatedElevatorConfiguration getSimulatedElevatorConfiguration() {
        return kElevatorConfiguration.kSimulatedElevatorConfiguration;
    }
}
