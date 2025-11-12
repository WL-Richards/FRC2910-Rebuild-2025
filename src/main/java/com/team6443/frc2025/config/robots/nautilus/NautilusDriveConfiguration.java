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
    
    private final String kDriveSubsystemName;
    private final String kDriveCANBusName;

    // --- Gyro ---
    public Pigeon2GyroConfiguration kGyro = 
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
                    "Pigeon2",
                    kDriveSubsystemName,
                    CANDeviceID.CANDeviceType.PIGEON2,
                    kDriveCANBusName
                )
            );

    
    // --- Swerve Modules Config ---
    private class Swerve {
        // --- Physical Properties ---
        private static final double kDriveFrictionVoltage = 0.25; // The minimum voltage required for the drive motor to begin moving
        private static final double kSteerFrictionVoltage = 0.001; // The minimum voltage required for the steer motor to begin moving

        private static final double kDriveInertia = 0.001; // The rotational inertia in the drive system (kg * m^2) and represent rotational resistance to acceleration
        private static final double kSteerInertia = 0.00001; // The rotational inertia in the drive system (kg * m^2) and represent rotational resistance to acceleration

        private static final double kWheelRadiusM = Units.inchesToMeters(1.9375);

        private static final NeutralMode kDriveNeutralMode = NeutralMode.BRAKE;
        private static final NeutralMode kSteerNeutralMode = NeutralMode.BRAKE;

        // --- Gearbox Configuration ---
        private static final MultistageGearBox kDriveGearBox = 
            new MultistageGearBox()
                .addStage(16, 50)
                .addStage(28, 16)
                .addStage(15, 45);

        private static final MultistageGearBox kSteerGearBox = 
            new MultistageGearBox()
                .addStage(7, 150);

        // --- Encoder Configurations ---
        // Offset from what the encoder thinks is 0 to the true zero of the module in ROTATIONS
        private static final double kFrontLeftEncoderOffsetRotations = -0.7915340865489908;
        private static final double kFrontRightEncoderOffsetRotations = -(-0.23316507975861744 + Math.PI);
        private static final double kBackLeftEncoderOffsetRotations = 0.09050486648525283;
        private static final double kBackRightEncoderOffsetRotations = 0.100830078125 - 0.5;

        // --- Motor Configurations ---
        // Motor Gains configured for the drive swerve motors
        private static final ConfigureSlot0Gains kDriveMotorGains = new ConfigureSlot0Gains(
            2.0, 
            0.0, 
            0.005, 
            0, // No gravity
            0.37914,
            2.0608994822,
            0.013797
        );

        // Motor Gains configured for the steer swerve motors
        private static final ConfigureSlot0Gains kSteerMotorGains = new ConfigureSlot0Gains(
            75,
            0.0,
            0.0,
            0.0,    // No gravity
            0.16677,
            2.5678,
            0.0
        );

        // Drive Motor
        private static final double kDriveMotorSupplyCurrentLimit = 50.0; // The amount of current (amps) that this motor is allowed to pull from the battery, if exceeded voltage will be reduced to avoid brownouts
        private static final double kDriveMotorStatorCurrentLimit = 80.0;  // The amount of current (amps) that motor is allowed to draw up to
        private static final double kDriveMotorSlipCurrent = 120; // The amount of current (amps) that can be applied to the drive wheel before it slips (120 basically means it doesn't slip)
        
        // --- Module Configurations ---
        private class Modules {
            // ----------- Front Left -----------
            private static final CANDeviceID kFrontLeftDriveMotor =  new CANDeviceID(
                7, 
                "FrontLeftSwerveDriveMotor", 
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                kDriveCANBusName
            );

            private static final CANDeviceID kFrontLeftSteerMotor = new CANDeviceID(
            8, 
                "FrontLeftSwerveSteerMotor", 
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                kDriveCANBusName
            );
            
            private static final CANDeviceID kFrontLeftSteerEncoder = new CANDeviceID(
            26, 
                "FrontLeftSwerveSteerEncoder", 
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER, 
                kDriveCANBusName
            );

            private static SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kFrontLeftModule = 
            new TalonFXSwerveModuleConfiguration(
                    "FrontLeftSwerveModule", 
                    kFrontLeftDriveMotor, 
                    kFrontLeftSteerEncoder, 
                    kFrontLeftSteerEncoder
                )
                .withDriveFrictionVoltage(kDriveFrictionVoltage)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveMotorInverted(true)
                .withSteerMotorInverted(true)
                .withDriveInertia(kDriveInertia)
                .withSteerInertia(kSteerInertia)
                .withDriveMotorGains(kDriveMotorGains)
                .withSteerMotorGains(kSteerMotorGains)
                .withDriveNeutralMode(kDriveNeutralMode)
                .withSteerNeutralMode(kSteerNeutralMode)
                .withEncoderOffsetRotations(kFrontLeftEncoderOffsetRotations)
                .withDriveMotorSupplyCurrentLimit(kDriveMotorSupplyCurrentLimit)
                .withDriveMotorStatorCurrentLimit(kDriveMotorStatorCurrentLimit)
                .withDriveMotorSlipCurrent(kDriveMotorSlipCurrent)
                .withDriveGearBox(kDriveGearBox)
                .withSteerGearBox(kSteerGearBox)
                .withDriveMotor(kFrontLeftDriveMotor)
                .withDriveMotorType(DCMotor.getKrakenX60(1))
                .withSteerMotor(kFrontLeftSteerMotor)
                .withSteerEncoder(kFrontLeftSteerEncoder)
                .withLocationOffset(kWheelBaseLengthM / 2, kWheelTrackWidthM / 2)
                .withWheelRadiusM(kWheelRadiusM);

            // ----------- Front Right -----------
            private static final CANDeviceID kFrontRightDriveMotor =  new CANDeviceID(
                5, 
                "FrontRightSwerveDriveMotor", 
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                kDriveCANBusName
            );

            private static final CANDeviceID kFrontRightSteerMotor = new CANDeviceID(
            6, 
                "FrontRightSwerveSteerMotor", 
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                kDriveCANBusName
            );

            private static final CANDeviceID kFrontRightSteerEncoder = new CANDeviceID(
            24, 
                "FrontRightSwerveSteerEncoder", 
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER, 
                kDriveCANBusName
            );

            private static SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kFrontRightModule = 
            new TalonFXSwerveModuleConfiguration(
                "FrontRightSwerveModule", 
                kFrontRightDriveMotor, 
                kFrontRightSteerMotor, 
                kFrontRightSteerEncoder
                )
                .withDriveFrictionVoltage(kDriveFrictionVoltage)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveMotorInverted(true)
                .withSteerMotorInverted(true)
                .withDriveInertia(kDriveInertia)
                .withSteerInertia(kSteerInertia)
                .withDriveMotorGains(kDriveMotorGains)
                .withSteerMotorGains(kSteerMotorGains)
                .withDriveNeutralMode(kDriveNeutralMode)
                .withSteerNeutralMode(kSteerNeutralMode)
                .withEncoderOffsetRotations(kFrontRightEncoderOffsetRotations)
                .withDriveMotorSupplyCurrentLimit(kDriveMotorSupplyCurrentLimit)
                .withDriveMotorStatorCurrentLimit(kDriveMotorStatorCurrentLimit)
                .withDriveMotorSlipCurrent(kDriveMotorSlipCurrent)
                .withDriveGearBox(kDriveGearBox)
                .withSteerGearBox(kSteerGearBox)
                .withDriveMotor(kFrontRightDriveMotor)
                .withDriveMotorType(DCMotor.getKrakenX60(1))
                .withSteerMotor(kFrontRightSteerMotor)
                .withSteerEncoder(kFrontRightSteerEncoder)
                .withLocationOffset(kWheelBaseLengthM / 2, -kWheelTrackWidthM / 2)
                .withWheelRadiusM(kWheelRadiusM);

            // ----------- Back Left -----------
            private static final CANDeviceID kBackLeftDriveMotor = new CANDeviceID(
                3, 
                "BackLeftSwerveDriveMotor", 
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                kDriveCANBusName
            );
            private static final CANDeviceID kBackLeftSteerMotor = new CANDeviceID(
            4, 
                "BackLeftSwerveSteerMotor", 
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                kDriveCANBusName
            );
            private static final CANDeviceID kBackLeftSteerEncoder = new CANDeviceID(
            25, 
                "BackLeftSwerveSteerEncoder", 
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER, 
                kDriveCANBusName
            );
            

            private static SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kBackLeftModule = 
            new TalonFXSwerveModuleConfiguration(
                "BackLeftSwerveModule", 
                kBackLeftDriveMotor, 
                kBackLeftSteerMotor, 
                kBackLeftSteerEncoder
                )
                .withDriveFrictionVoltage(kDriveFrictionVoltage)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveMotorInverted(true)
                .withSteerMotorInverted(true)
                .withDriveInertia(kDriveInertia)
                .withSteerInertia(kSteerInertia)
                .withDriveMotorGains(kDriveMotorGains)
                .withSteerMotorGains(kSteerMotorGains)
                .withDriveNeutralMode(kDriveNeutralMode)
                .withSteerNeutralMode(kSteerNeutralMode)
                .withEncoderOffsetRotations(kBackLeftEncoderOffsetRotations)
                .withDriveMotorSupplyCurrentLimit(kDriveMotorSupplyCurrentLimit)
                .withDriveMotorStatorCurrentLimit(kDriveMotorStatorCurrentLimit)
                .withDriveMotorSlipCurrent(kDriveMotorSlipCurrent)
                .withDriveGearBox(kDriveGearBox)
                .withSteerGearBox(kSteerGearBox)
                .withDriveMotor(kBackLeftDriveMotor)
                .withDriveMotorType(DCMotor.getKrakenX60(1))
                .withSteerMotor(kBackLeftSteerMotor)
                .withSteerEncoder(kBackLeftSteerEncoder)
                .withLocationOffset(-kWheelBaseLengthM / 2, kWheelTrackWidthM / 2)
                .withWheelRadiusM(kWheelRadiusM);

            // Back Right Swerve Module Configuration
            private static final CANDeviceID kBackRightDriveMotor = new CANDeviceID(
                9, 
                "BackRightSwerveDriveMotor", 
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                kDriveCANBusName
            );

            private static final CANDeviceID kBackRightSteerMotor = new CANDeviceID(
                2, 
                "BackRightSwerveSteerMotor",  
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                kDriveCANBusName
            );

            private static final CANDeviceID kBackRightSteerEncoder = new CANDeviceID(
                23, 
                "BackRightSwerveSteerEncoder",  
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.CANCODER, 
                kDriveCANBusName
            );

            private static SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kBackRightModule = 
                new TalonFXSwerveModuleConfiguration(
                    "BackRightSwerveModule", 
                    kBackRightDriveMotor, 
                    kBackRightSteerMotor, 
                    kBackRightSteerEncoder
                    )
                    .withDriveFrictionVoltage(kDriveFrictionVoltage)
                    .withSteerFrictionVoltage(kSteerFrictionVoltage)
                    .withDriveMotorInverted(false)
                    .withSteerMotorInverted(true)
                    .withDriveInertia(kDriveInertia)
                    .withSteerInertia(kSteerInertia)
                    .withDriveMotorGains(kDriveMotorGains)
                    .withSteerMotorGains(kSteerMotorGains)
                    .withDriveNeutralMode(kDriveNeutralMode)
                    .withSteerNeutralMode(kSteerNeutralMode)
                    .withEncoderOffsetRotations(kBackRightEncoderOffsetRotations)
                    .withDriveMotorSupplyCurrentLimit(kDriveMotorSupplyCurrentLimit)
                    .withDriveMotorStatorCurrentLimit(kDriveMotorStatorCurrentLimit)
                    .withDriveMotorSlipCurrent(kDriveMotorSlipCurrent)
                    .withDriveGearBox(kDriveGearBox)
                    .withSteerGearBox(kSteerGearBox)
                    .withDriveMotor(kBackRightDriveMotor)
                    .withDriveMotorType(DCMotor.getKrakenX60(1))
                    .withSteerMotor(kBackRightSteerMotor)
                    .withSteerEncoder(kBackRightSteerEncoder)
                    .withLocationOffset(-kWheelBaseLengthM / 2, kWheelTrackWidthM / 2)
                    .withWheelRadiusM(kWheelRadiusM);
        }
    }

    // --- Odometry Config ---
    private class Odometry{
        private static final OdometryStandardDevs kDisabledModeStandardDevs = new OdometryStandardDevs(1, 1, 1);

        private static final OdometryStandardDevs kEnabledModeStandardDevs = new OdometryStandardDevs(0.3, 0.3, 0.2);
    }
    
    private static final double kChassisTranslationSpeedThreshold = 0.05; // Anything less than this chassis speed in meters per second will be set to 0
    private static final double kChassisRotationalSpeedThreshold = 0.05; // Anything less than this chassis speed in radians per second will be set to 0
    private static final double kMaxDriveSpeed = 3.6; // Max speed of the robot in meters per second
    public static final double kMaxAngularRate = 8.2; // Radians per second

    private static final DrivetrainConfiguration kDrivetrainConfiguration = 
        new DrivetrainConfiguration()
                .withName(kDriveSubsystemName)
                .withMaxDriveSpeed(kMaxDriveSpeed)
                .withMaxAngularRate(kMaxAngularRate)
                .withChassisSpeedDeadband(
                    kChassisTranslationSpeedThreshold, 
                    kChassisRotationalSpeedThreshold
                )
                .withDrivetrainConstants(
                    new SwerveDrivetrainConstants()
                        .withCANBusName(kDriveCANBusName)
                        .withPigeon2Id(Drive.Gyro.kID.getDeviceID())
                        .withPigeon2Configs(kGyro.kConfiguration)
                )
                // .withDrivetrainConstants(NautilusSwerveConstantsComp.DrivetrainConstants) // Phoenix Tuner Supplied Constants
                .withModuleConstants(
                    new SwerveModuleConstants<?, ?, ?>[]{
                        Swerve.Modules.kFrontLeftModule.getModuleConstants(),
                        Swerve.Modules.kFrontRightModule.getModuleConstants(),
                        Swerve.Modules.kBackLeftModule.getModuleConstants(),
                        Swerve.Modules.kBackRightModule.getModuleConstants()
                    }
                )
                // .withModuleConstants(NautilusSwerveConstantsComp.kSwerveModuleConstants) // Phoenix Tuner Supplied Constants
                .withOdometryStandardDevs(
                    Odometry.kEnabledModeStandardDevs,
                    Odometry.kDisabledModeStandardDevs
                )
                .withJoystickDeadband(0.05, 0.05);

    private static final DrivetrainSimConfiguration kSimulatedDrivetrainConfiguration = 
    new DrivetrainSimConfiguration(0.005) // 5 ms
            .withName(kDriveSubsystemName)
            .withRobotWeightPounds(150)
            .withBumperLengthInches(35.625)
            .withBumperWidthInches(35.625)
            .withWheelCoefficientOfFriction(1.2)
            .withDriveMotorCount(1)
            .withSteerMotorCount(1);

    public NautilusDriveConfiguration(String driveSubsystemName, String driveSubsystemCANBus){
        this.kDriveSubsystemName = driveSubsystemName;
        this.kDriveCANBusName = driveSubsystemCANBus;
    }
}
