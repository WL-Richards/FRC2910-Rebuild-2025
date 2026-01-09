// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.config.robots.nautilus;

import java.util.List;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6443.frc2025.config.RobotConfig;
import com.team6443.lib.config.autonomous.ChoreoPatherConfiguration;
import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.config.camera.SimulatedCameraConfiguration;
import com.team6443.lib.config.motors.MotorFollowerConfiguration;
import com.team6443.lib.config.robot.PhysicalConfiguration;
import com.team6443.lib.config.robot.Pigeon2GyroConfiguration;
import com.team6443.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.team6443.lib.config.subsystems.drive.simulation.DrivetrainSimConfiguration;
import com.team6443.lib.config.subsystems.elevator.simulation.SimulatedElevatorConfiguration;
import com.team6443.lib.config.superstructure.BaseSuperstructureConfiguration;
import com.team6443.lib.config.swerve.SwerveModuleConfiguration;
import com.team6443.lib.core.can.CANDeviceID;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/**
 * This code represents the configuration for 6443's 2025 robot "Nautilus" 
 */
public class Nautilus extends RobotConfig {

    // --- Misc. Config ---
    private static final String kRobotName = "Nautilus";

    private static final String kElevatorSubsystemName = "ElevatorSubsystem";
    private static final String kDriveSubsystemName = "DriveSubsystem";

    // --- Superstructure Config ---
    private static final NautilusSuperstructureConfiguration kSuperstructureConfiguration = 
        new NautilusSuperstructureConfiguration(kRobotName + "_DefaultConfiguration");

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
    public ChoreoPatherConfiguration getChoreoPatherConfiguration() {
        return kAutonomousConfiguration.kChoreoPatherConfiguration;
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
    public MotorFollowerConfiguration<TalonFXConfiguration> getElevatorConfiguration() {
        return kElevatorConfiguration.kElevatorMotorSubsystemConfiguration;
    }

    @Override
    public SimulatedElevatorConfiguration getSimulatedElevatorConfiguration() {
        return kElevatorConfiguration.kSimulatedElevatorConfiguration;
    }

    @Override
    public BaseSuperstructureConfiguration<?> getSuperstructureConfiguration() {
        return kSuperstructureConfiguration;
    }
}
