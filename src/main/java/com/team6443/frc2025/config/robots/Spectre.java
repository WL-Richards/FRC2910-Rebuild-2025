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
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.team6443.frc2025.config.RobotConfig;
import com.team6443.lib.can.CANDeviceID;
import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.config.camera.CameraConfiguration.Location;
import com.team6443.lib.config.motors.ServoMotorCANCoderConfiguration;
import com.team6443.lib.config.swerve.SwerveModuleConfiguration;
import com.team6443.lib.config.swerve.TalonFXSwerveModuleConfiguration;
import com.team6443.lib.config.wrappers.ConfigureSlot0Gains;
import com.team6443.lib.factories.motors.TalonFXFactory;
import com.team6443.lib.mechanics.MultistageGearBox;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * This code represents the configuration for 2910's 2025 spectre robot
 */
public class Spectre extends RobotConfig {

    // --- Robot Configuration Settings
    private final List<CameraConfiguration> cameraConfigurations;
    private final List<SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>> moduleConstants;
    private final SwerveDrivetrainConstants swerveDriveConstants;

    private final ServoMotorCANCoderConfiguration<TalonFXConfiguration> testSubsystemConfig;
    static {
       
    }
    
    // --- Physical Properties --- 
    // Radius of the modules wheel in meters
    private static final double kWheelBaseLengthM = Units.inchesToMeters(22.75);
    private static final double kWheelTrackWidthM = Units.inchesToMeters(20.75);

    // ---  CAN Bus Config  --- 
    // (Note IDs can be the same for different device types (TalonFX, Pigeon2, etc), ie gyro and front left drive motor have the same ID)
    private static final String kCanivoreBusName = "CANivore";

    // List of can buses that are present on this robot
    private static final List<String> kCANBuses = List.of(
        kCanivoreBusName
    );


    // --- Misc. Config ---
    private static final String kRobotName = "Spectre";

    private static final String kDriveSubsystemName = "DriveSubsystem";
    private static final String kTestSubsystemName = "TestSubsystem";
    
    // --- Gyro Config ---
    private class Gyro {
        /**
         * The error of the gyro relative to what we determine is true 
         * 
         * To determine face robot in a direction that we know what the angle should be, GyroYawErrorDegrees = (gyro reading - true reading)
         */
        private static final double kGyroYawErrorDegrees = -0.34;

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

        private static final double kWheelRadiusM = Units.inchesToMeters(1.95);

        // --- Gearbox Configuration ---
        private static final MultistageGearBox kDriveGearBox = 
            new MultistageGearBox()
                .addStage(12, 54)
                .addStage(32, 25)
                .addStage(15, 30);

        private static final MultistageGearBox kSteerGearBox = 
            new MultistageGearBox()
                .addStage(12, 54)
                .addStage(32, 25)
                .addStage(15, 30);

        // --- Encoder Configurations ---
        // Offset from what the encoder thinks is 0 to the true zero of the module in ROTATIONS
        private static final double kFrontLeftEncoderOffsetRotations = -0.30517578125 + 0.5;
        private static final double kFrontRightEncoderOffsetRotations = -0.008544921875;
        private static final double kBackLeftEncoderOffsetRotations = -0.341064453125 + 0.5;
        private static final double kBackRightEncoderOffsetRotations = 0.100830078125 - 0.5;

        // --- Motor Configurations ---
        // Motor Gains configured for the drive swerve motors
        private static final ConfigureSlot0Gains kDriveMotorGains = new ConfigureSlot0Gains(
            0.0, 
            0.0, 
            0.0, 
            0.1238, 
            0.0
        );

        // Motor Gains configured for the steer swerve motors
        private static final ConfigureSlot0Gains kSteerMotorGains = new ConfigureSlot0Gains(
            100.0,
            0.0,
            0.0,
            0.0,
            0.0
        );

        // Drive Motor
        private static final double kDriveMotorSupplyCurrentLimit = 50.0; // The amount of current (amps) that this motor is allowed to pull from the battery, if exceeded voltage will be reduced to avoid brownouts
        private static final double kDriveMotorStatorCurrentLimit = 100.0;  // The amount of current (amps) that motor is allowed to draw up to
        private static final double kDriveMotorSlipCurrent = 120; // The amount of current (amps) that can be applied to the drive wheel before it slips (120 basically means it doesn't slip)

        // --- Module Configurations ---
        private class Modules {
            // Front Left
            private static final CANDeviceID kFrontLeftDriveMotor =  new CANDeviceID(
                1, 
                "FrontLeftSwerveDriveMotor", 
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                kCanivoreBusName
            );

            private static final CANDeviceID kFrontLeftSteerMotor = new CANDeviceID(
            2, 
                "FrontLeftSwerveSteerMotor", 
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                kCanivoreBusName
            );
            
            private static final CANDeviceID kFrontLeftSteerEncoder = new CANDeviceID(
            1, 
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
                .withDriveInertia(kDriveInertia)
                .withSteerInertia(kSteerInertia)
                .withDriveMotorGains(kDriveMotorGains)
                .withSteerMotorGains(kSteerMotorGains)
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

            // Front Right
            private static final CANDeviceID kFrontRightDriveMotor =  new CANDeviceID(
                3, 
                "FrontRightSwerveDriveMotor", 
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                kCanivoreBusName
            );

            private static final CANDeviceID kFrontRightSteerMotor = new CANDeviceID(
            4, 
                "FrontRightSwerveSteerMotor", 
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                kCanivoreBusName
            );

            private static final CANDeviceID kFrontRightSteerEncoder = new CANDeviceID(
            2, 
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
                .withDriveInertia(kDriveInertia)
                .withSteerInertia(kSteerInertia)
                .withDriveMotorGains(0.0, 0.0, 0.0, 0.1238, 0.0)
                .withSteerMotorGains(100.0, 0.0, 0.0, 0.0, 0.0)
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

            // Back Left
            private static final CANDeviceID kBackLeftDriveMotor = new CANDeviceID(
                5, 
                "BackLeftSwerveDriveMotor", 
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                kCanivoreBusName
            );
            private static final CANDeviceID kBackLeftSteerMotor = new CANDeviceID(
            6, 
                "BackLeftSwerveSteerMotor", 
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                kCanivoreBusName
            );
            private static final CANDeviceID kBackLeftSteerEncoder = new CANDeviceID(
            3, 
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
                .withDriveInertia(kDriveInertia)
                .withSteerInertia(kSteerInertia)
                .withDriveMotorGains(0.0, 0.0, 0.0, 0.1238, 0.0)
                .withSteerMotorGains(100.0, 0.0, 0.0, 0.0, 0.0)
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
                7, 
                "BackRightSwerveDriveMotor", 
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                kCanivoreBusName
            );

            private static final CANDeviceID kBackRightSteerMotor = new CANDeviceID(
                8, 
                "BackRightSwerveSteerMotor",  
                kDriveSubsystemName,
                CANDeviceID.CANDeviceType.TALON_FX, 
                kCanivoreBusName
            );

            private static final CANDeviceID kBackRightSteerEncoder = new CANDeviceID(
                8, 
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
                    .withDriveInertia(kDriveInertia)
                    .withSteerInertia(kSteerInertia)
                    .withDriveMotorGains(0.0, 0.0, 0.0, 0.1238, 0.0)
                    .withSteerMotorGains(100.0, 0.0, 0.0, 0.0, 0.0)
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
    
    // --- Camera Config ---
    private class Cameras {
        
        // Front Left Camera
        private static final CameraConfiguration kFrontLeftCameraConfiguration = new CameraConfiguration(Location.FRONT_LEFT)
        .withCameraPose(
            new Translation3d(
                Units.inchesToMeters(8.479),    // X (forward)
                Units.inchesToMeters(-6.75),           // Y (right)
                Units.inchesToMeters(8.479)     // Z (up)
            ),
            new Rotation3d(
                0,      // Roll
                0,     // Pitch
                0        // Yaw
            )
        )
        .withCameraDistanceScalar(24.25, 24.06)
        .withCameraType(CameraConfiguration.Type.LIMELIGHT);

        // Front Right Camera
        private static final CameraConfiguration kFrontRightCameraConfiguration = new CameraConfiguration(Location.FRONT_RIGHT)
            .withCameraPose(
                new Translation3d(
                    Units.inchesToMeters(8.479),    // X (forward)
                    Units.inchesToMeters(6.75),     // Y (right)
                    Units.inchesToMeters(8.479)     // Z (up)
                ),
                new Rotation3d(
                    0,      // Roll
                    0,     // Pitch
                    0        // Yaw
                )
            )
            .withCameraDistanceScalar(24.25, 24.24)
            .withCameraType(CameraConfiguration.Type.LIMELIGHT);

        // Back Left Camera
        private static final CameraConfiguration kBackLeftCameraConfiguration = new CameraConfiguration(Location.BACK_LEFT)
            .withCameraPose(
                new Translation3d(
                    Units.inchesToMeters(11.199),   // X (forward)
                    Units.inchesToMeters(-6.75),           // Y (right)
                    Units.inchesToMeters(8.440)     // Z (up)
                ),
                new Rotation3d(
                    0,      // Roll
                    0,     // Pitch
                    180      // Yaw
                )
            )
            .withCameraDistanceScalar(10.25, 10.77)
            .withCameraType(CameraConfiguration.Type.LIMELIGHT);

        // Back Right Camera 
        private static final CameraConfiguration kBackRightCameraConfiguration = new CameraConfiguration(Location.BACK_RIGHT)
        .withCameraPose(
            new Translation3d(
                Units.inchesToMeters(11.199),   // X (forward)
                Units.inchesToMeters(6.75),     // Y (right)
                Units.inchesToMeters(8.440)     // Z (up)
            ),
            new Rotation3d(
                0,      // Roll
                0,     // Pitch
                180      // Yaw
            )
        )
        .withCameraDistanceScalar(10.25, 10.88)
        .withCameraType(CameraConfiguration.Type.LIMELIGHT);
    }
    
    /**
     * Constructor for this robot configuration sets up variables that are used in the RobotConfig's override functions
     */
    public Spectre(){
        // Configure the cameras on this bot
        cameraConfigurations = buildCameraConfigurations();

        // Construct our swerve module representations
        moduleConstants = buildSwerveModuleConstants();

        // Construct our constants for the overall swerve drive
        swerveDriveConstants = buildSwerveDriveConstants();

        testSubsystemConfig = buildTestSubsystemConfiguration();
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
     * Constructs and returns a list of swerve modules constants (each element = one module (in LF, RF, BL, BR order)) utilized on this robot config
     * @return List of swerve module constants
     */
    private List<SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>> buildSwerveModuleConstants(){
        return List.of(
            Swerve.Modules.kFrontLeftModule.getModuleConstants(),
            Swerve.Modules.kFrontRightModule.getModuleConstants(),
            Swerve.Modules.kBackLeftModule.getModuleConstants(),
            Swerve.Modules.kBackRightModule.getModuleConstants()
        );
    }

    /**
     * Build and return a set of swerve drive constants for this class
     * @return
     */
    private SwerveDrivetrainConstants buildSwerveDriveConstants(){
        return new SwerveDrivetrainConstants()
            .withCANBusName(kCanivoreBusName)
            .withPigeon2Id(Gyro.kID.getDeviceID())
            .withPigeon2Configs(Gyro.kConfiguration);
    }

    /**
     * Build and return a set of swerve drive constants for this class
     * @return
     */
    private ServoMotorCANCoderConfiguration<TalonFXConfiguration> buildTestSubsystemConfiguration(){
        ServoMotorCANCoderConfiguration<TalonFXConfiguration> config = new ServoMotorCANCoderConfiguration<TalonFXConfiguration>()
                                                                                .withConfig(new TalonFXConfiguration());


        config.ConfigurationName = "TestSubsystem";
        config.CANDevice = new CANDeviceID(
            14, 
            "Flywheel",  
            kTestSubsystemName,
            CANDeviceID.CANDeviceType.TALON_FX, 
            kCanivoreBusName
        );

        config.momentOfInertia = 0.02330333;
        config.unitToRotorRotationRatio = Units.rotationsToRadians(1);

        config.motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.canCoderConfig.CANDevice = new CANDeviceID(
            19, 
            "Flywheel",  
            kTestSubsystemName,
            CANDeviceID.CANDeviceType.CANCODER, 
            kCanivoreBusName
        );

        config.canCoderConfig.config.MagnetSensor.MagnetOffset = 0.;
        config.CANCoderRotationToUnitRatio = Units.rotationsToRadians(1);

        config.motorConfig.Feedback.FeedbackRemoteSensorID = config.canCoderConfig.CANDevice.getDeviceID();
        config.motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.motorConfig.Feedback.SensorToMechanismRatio = 1 / 1.0;

        config.motorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        config.motorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CANCoderGearRatioSim = 1.0;
        config.CANCoderUnitToRotorRotationRatioSim = 1.0;

        return config;
    }

    // --- RobotConfig override functions ---
    @Override
    public SwerveDrivetrainConstants getSwerveDriveConstants() {
        return swerveDriveConstants;
    }

    @Override
    public List<SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>> getModuleConstants() {
        return moduleConstants;
    }


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
    public ServoMotorCANCoderConfiguration<TalonFXConfiguration> getTestSubsystemConfiguration() {
        return testSubsystemConfig;
    }
}
