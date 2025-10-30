// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.config.robots;

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
import com.team6443.lib.can.CANDeviceID;
import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.config.camera.CameraConfiguration.Location;
import com.team6443.lib.config.motors.ServoMotorFollowerConfiguration;
import com.team6443.lib.config.motors.factories.TalonFXConfigurationFactory;
import com.team6443.lib.config.odometry.OdometryStandardDevs;
import com.team6443.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.team6443.lib.config.subsystems.drive.DrivetrainSimConfiguration;
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

    // --- Robot Configuration Settings
    private final List<CameraConfiguration> cameraConfigurations;
    private final ServoMotorFollowerConfiguration<TalonFXConfiguration> elevatorSubsystemConfig;
    private final SimulatedElevatorConfiguration simulatedElevatorConfiguration;
    
    // --- Physical Properties --- 
    // Radius of the modules wheel in meters
    private static final double kWheelBaseLengthM = Units.inchesToMeters(22.75);
    private static final double kWheelTrackWidthM = Units.inchesToMeters(22.75);

    // ---  CAN Bus Config  --- 
    // (Note IDs can be the same for different device types (TalonFX, Pigeon2, etc), ie gyro and front left drive motor have the same ID)
    private static final String kCanivoreBusName = "CANivore";

    // List of can buses that are present on this robot
    private static final List<String> kCANBuses = List.of(
        kCanivoreBusName
    );


    // --- Misc. Config ---
    private static final String kRobotName = "Nautilus";

    private static final String kDriveSubsystemName = "DriveSubsystem";
    private static final String kElevatorSubsystemName = "ElevatorSubsystem";
    
    // --- Drivetrain Config ---
    private class Drive {
        // --- Gyro Config ---
        private class Gyro {
            /**
             * The error of the gyro relative to what we determine is true 
             * 
             * To determine face robot in a direction that we know what the angle should be, GyroYawErrorDegrees = (gyro reading - true reading)
             */
            private static final double kGyroYawErrorDegrees = 0.0;

            // Rotation that the gyro is mounted all degrees passed in must be converted to radians
            private static final Rotation3d kMountRotation = new Rotation3d(
            Math.toRadians(0),      // Roll
            Math.toRadians(0),      // Pitch
            Math.toRadians(0)       // Yaw
            );

            // CAN Device ID associated with the gyro
            private static final CANDeviceID kID = new CANDeviceID(
                1, 
                "Pigeon2",
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.PIGEON2,
                kCanivoreBusName
            );

            // Configuration for the Pigeon2 that is to be used with the swerve drive
            private static final Pigeon2Configuration kConfiguration = new Pigeon2Configuration()
            .withMountPose(
                new MountPoseConfigs()
                    .withMountPoseRoll(Math.toDegrees(kMountRotation.getX()))
                    .withMountPosePitch(Math.toDegrees(kMountRotation.getY()))
                    .withMountPoseYaw(Math.toDegrees(kMountRotation.getZ()))
            )
            .withGyroTrim(
                new GyroTrimConfigs()
                    .withGyroScalarZ(kGyroYawErrorDegrees)
            );
        }
        
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
                    kCanivoreBusName
                );

                private static final CANDeviceID kFrontLeftSteerMotor = new CANDeviceID(
                8, 
                    "FrontLeftSwerveSteerMotor", 
                    kDriveSubsystemName,
                    CANDeviceID.CANDeviceType.TALON_FX, 
                    kCanivoreBusName
                );
                
                private static final CANDeviceID kFrontLeftSteerEncoder = new CANDeviceID(
                26, 
                    "FrontLeftSwerveSteerEncoder", 
                    kDriveSubsystemName,
                    CANDeviceID.CANDeviceType.CANCODER, 
                    kCanivoreBusName
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
                    kCanivoreBusName
                );

                private static final CANDeviceID kFrontRightSteerMotor = new CANDeviceID(
                6, 
                    "FrontRightSwerveSteerMotor", 
                    kDriveSubsystemName,
                    CANDeviceID.CANDeviceType.TALON_FX, 
                    kCanivoreBusName
                );

                private static final CANDeviceID kFrontRightSteerEncoder = new CANDeviceID(
                24, 
                    "FrontRightSwerveSteerEncoder", 
                    kDriveSubsystemName,
                    CANDeviceID.CANDeviceType.CANCODER, 
                    kCanivoreBusName
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
                    kCanivoreBusName
                );
                private static final CANDeviceID kBackLeftSteerMotor = new CANDeviceID(
                4, 
                    "BackLeftSwerveSteerMotor", 
                    kDriveSubsystemName,
                    CANDeviceID.CANDeviceType.TALON_FX, 
                    kCanivoreBusName
                );
                private static final CANDeviceID kBackLeftSteerEncoder = new CANDeviceID(
                25, 
                    "BackLeftSwerveSteerEncoder", 
                    kDriveSubsystemName,
                    CANDeviceID.CANDeviceType.CANCODER, 
                    kCanivoreBusName
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
                    kCanivoreBusName
                );

                private static final CANDeviceID kBackRightSteerMotor = new CANDeviceID(
                    2, 
                    "BackRightSwerveSteerMotor",  
                    kDriveSubsystemName,
                    CANDeviceID.CANDeviceType.TALON_FX, 
                    kCanivoreBusName
                );

                private static final CANDeviceID kBackRightSteerEncoder = new CANDeviceID(
                    23, 
                    "BackRightSwerveSteerEncoder",  
                    kDriveSubsystemName,
                    CANDeviceID.CANDeviceType.CANCODER, 
                    kCanivoreBusName
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

        private static final DrivetrainConfiguration kDrivetrainConfiguration = new DrivetrainConfiguration()
                                                                                        .withName(kDriveSubsystemName)
                                                                                        .withMaxDriveSpeed(kMaxDriveSpeed)
                                                                                        .withChassisSpeedDeadband(
                                                                                            kChassisTranslationSpeedThreshold, 
                                                                                            kChassisRotationalSpeedThreshold
                                                                                        )
                                                                                        .withDrivetrainConstants(
                                                                                            new SwerveDrivetrainConstants()
                                                                                                .withCANBusName(kCanivoreBusName)
                                                                                                .withPigeon2Id(Drive.Gyro.kID.getDeviceID())
                                                                                                .withPigeon2Configs(Drive.Gyro.kConfiguration)
                                                                                        )
                                                                                        .withModuleConstants(
                                                                                            new SwerveModuleConstants<?, ?, ?>[]{
                                                                                                Drive.Swerve.Modules.kFrontLeftModule.getModuleConstants(),
                                                                                                Drive.Swerve.Modules.kFrontRightModule.getModuleConstants(),
                                                                                                Drive.Swerve.Modules.kBackLeftModule.getModuleConstants(),
                                                                                                Drive.Swerve.Modules.kBackRightModule.getModuleConstants()
                                                                                            }
                                                                                        )
                                                                                        .withOdometryStandardDevs(
                                                                                            Odometry.kEnabledModeStandardDevs,
                                                                                            Odometry.kDisabledModeStandardDevs
                                                                                        );

        private static final DrivetrainSimConfiguration kSimulatedDrivetrainConfiguration = new DrivetrainSimConfiguration(0.005) // 5 ms
                                                                                        .withName(kDriveSubsystemName)
                                                                                        .withRobotWeightPounds(150)
                                                                                        .withBumperLengthInches(35.625)
                                                                                        .withBumperWidthInches(35.625)
                                                                                        .withWheelCoefficientOfFriction(1.2)
                                                                                        .withDriveMotorCount(1)
                                                                                        .withSteerMotorCount(1);
       
    }
   
    // --- Camera Config ---
    private class Cameras {
        
        // Front Left Camera
        private static final CameraConfiguration kFrontLeftCameraConfiguration = new CameraConfiguration(Location.FRONT_LEFT)
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

        // Front Right Camera
        private static final CameraConfiguration kFrontRightCameraConfiguration = new CameraConfiguration(Location.FRONT_RIGHT)
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

        // Back Left Camera
        private static final CameraConfiguration kBackLeftCameraConfiguration = new CameraConfiguration(Location.BACK_LEFT)
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

        // Back Right Camera 
        private static final CameraConfiguration kBackRightCameraConfiguration = new CameraConfiguration(Location.BACK_RIGHT)
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
    }
    
    // --- Elevator Config ---
    private class Elevator {

        // The max height the elevator can extend to in meters
        private static final double kMaxHeight = 1.15;

        // The min height the elevator can retract to in meters
        private static final double kMinHeight = 0;

        // The max velocity that we want the elevator to be capable of m/s
        private static final double kMaxVelocity = 3.6;

        // The max acceleration that we want the elevator to be capable of m/s²
        private static final double kMaxAcceleration = 4.0;

        // The max acceleration that we want the elevator to be capable of m/s³
        private static final double kJerk = 0.0;

        // ID of the the leader elevator motor
        private static final int kTopLeaderMotorID = 13;
        private static final double kTopLeaderMotorCurrentLimit = 50.0;

        // ID of the the follower elevator motor
        private static final int kBottomFollowerMotorID = 12;
        private static final double kBottomFollowerMotorCurrentLimit = 50.0;

        public static final double kElevatorDrumRadius = 0.02866242038;
        public static final double kGearing = (11.0 / 50.0);
        public static final double kElevatorUnitToRotorRatio = kGearing * 2.0 * kElevatorDrumRadius * Math.PI;


        // Motion magic gains for the elevator
        private static final ConfigureSlot0Gains kMotorGains = new ConfigureSlot0Gains(
            1.0,
            0.0,
            0.0,
            0.325,
            0.075,
            0.1333,
            0.005
            )
        .withGravityType(GravityTypeValue.Elevator_Static);
        // private static final ConfigureSlot0Gains kMotorGains = new ConfigureSlot0Gains(
        //     10,
        //     0.0,
        //     0.0,
        //     0.35,
        //     0.15,
        //     0,
        //     0.005
        //     )
        // .withGravityType(GravityTypeValue.Elevator_Static);
    }

    
    /**
     * Constructor for this robot configuration sets up variables that are used in the RobotConfig's override functions
     */
    public Nautilus(){
        // Configure the cameras on this bot
        cameraConfigurations = buildCameraConfigurations();

        // Build the config for the elevator on this bot
        elevatorSubsystemConfig = buildElevatorConfig();
        simulatedElevatorConfiguration = buildSimulatedElevatorConfig(); 
    }

    /**
     * Construct a list of the camera configurations for this bot
     * @return A list of camera configurations
     */
    private List<CameraConfiguration> buildCameraConfigurations() {
        return List.of(
            Cameras.kFrontLeftCameraConfiguration,
            Cameras.kFrontRightCameraConfiguration,           
            Cameras.kBackLeftCameraConfiguration,
            Cameras.kBackRightCameraConfiguration
        );
        
    }

    /**
     * Build and return a new elevator config for this robot
     * @return The constructed config
     */
    private ServoMotorFollowerConfiguration<TalonFXConfiguration> buildElevatorConfig(){

        // ------------------------- Elevator Follower Motor Configuration----------------------------
        ServoMotorFollowerConfiguration.FollowerConfiguration<TalonFXConfiguration> followerConfig = TalonFXConfigurationFactory.generateFollowerTalonFXConfiguration();

        followerConfig.config.kConfigurationName = "BottomMotorFollower";
        followerConfig.config.kCANDevice = new CANDeviceID(
            Elevator.kBottomFollowerMotorID, 
            "BottomMotorFollower",  
            kElevatorSubsystemName,
            CANDeviceID.CANDeviceType.TALON_FX, 
            kCanivoreBusName
        );
        followerConfig.config.kUnitToRotorRotationRatio = Elevator.kElevatorUnitToRotorRatio;

        // Setup current limit on follower
        followerConfig.config.kMotorConfig.CurrentLimits.StatorCurrentLimit = Elevator.kBottomFollowerMotorCurrentLimit;
        followerConfig.config.kMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Set follow direction and motor NeutralMode
        followerConfig.followDirection = FollowDirection.SAME;
        followerConfig.config.kMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        // ------------------------- Elevator Subsystem Configuration -------------------------
        ServoMotorFollowerConfiguration<TalonFXConfiguration> config = new ServoMotorFollowerConfiguration<>(
                                                                            new TalonFXConfiguration()
                                                                        );
        config.kConfigurationName = "ElevatorSubsystem";
        config.kCANDevice = new CANDeviceID(
            Elevator.kTopLeaderMotorID, 
            "TopMotorLeader",  
            kElevatorSubsystemName,
            CANDeviceID.CANDeviceType.TALON_FX, 
            kCanivoreBusName
        );

        // Configure Elevator motor gains
        config.kMotorConfig.Slot0 = Elevator.kMotorGains;

        // Configure elevator rotor ratio
        config.kUnitToRotorRotationRatio = Elevator.kElevatorUnitToRotorRatio;

        // Configure elevator motion magic parameters
        config.kMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 90.0;
        config.kMotorConfig.MotionMagic.MotionMagicAcceleration = 1000.0;
        config.kMotorConfig.MotionMagic.MotionMagicJerk = 3600.0;

        // config.motorConfig.MotionMagic.MotionMagicCruiseVelocity = Elevator.kMaxVelocity / config.unitToRotorRotationRatio;
        // config.motorConfig.MotionMagic.MotionMagicAcceleration = Elevator.kMaxAcceleration / config.unitToRotorRotationRatio;
        // config.motorConfig.MotionMagic.MotionMagicJerk = Elevator.kJerk / config.unitToRotorRotationRatio;

        // Configure motor in brake mode
        config.kMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.kMotorConfig.CurrentLimits.StatorCurrentLimit = Elevator.kTopLeaderMotorCurrentLimit;
        config.kMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        config.kMaxPositionUnits = Elevator.kMaxHeight;
        config.kMinPositionUnits = Elevator.kMinHeight;

        // Follower configurations 
        config.followerConfigurations = List.of(followerConfig);

        return config;
    }

    private SimulatedElevatorConfiguration buildSimulatedElevatorConfig(){
        SimulatedElevatorConfiguration config = new SimulatedElevatorConfiguration();
        config.carriageMass = 1.97312681;
        config.drumRadius = Elevator.kElevatorDrumRadius;
        config.gearing = Elevator.kGearing;
        config.meterToRotorRatio = Elevator.kElevatorUnitToRotorRatio;
        return config;
    }

    // --- RobotConfig override functions ---
    @Override
    public String getRobotName() {
        return kRobotName;
    }

    @Override
    public List<CameraConfiguration> getCameraConfigurations() {
        return cameraConfigurations;
    }

    @Override
    public List<String> getCANBusNames() {
        return kCANBuses;
    }
 
    @Override
    public DrivetrainConfiguration getDrivetrainConfiguration() {
        return Drive.kDrivetrainConfiguration;
    }

    @Override
    public DrivetrainSimConfiguration getSimulatedDrivetrainConfiguration() {
        return Drive.kSimulatedDrivetrainConfiguration;
    }

    @Override
    public ServoMotorFollowerConfiguration<TalonFXConfiguration> getElevatorConfiguration() {
        return elevatorSubsystemConfig;
    }

    @Override
    public SimulatedElevatorConfiguration getSimulatedElevatorConfiguration() {
        return simulatedElevatorConfiguration;
    }

  
}
