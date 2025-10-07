// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import java.util.List;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.base.CameraConfiguration;
import frc.robot.config.base.PortConfiguration;
import frc.robot.config.base.RobotConfig;
import frc.robot.config.base.wrappers.ConfigureSlot0Gains;
import frc.robot.config.base.CameraConfiguration.Location;
import frc.robot.util.can.CANDeviceID;

/**
 * This code represents the configuration for 2910's 2025 spectre robot
 */
public class Spectre extends RobotConfig {

    // --- Robot Configuration Settings
    private final PortConfiguration m_portConfiguration;

    private final List<CameraConfiguration> m_cameraConfigurations;

    private final  List<SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>> m_moduleConstants;

    private final  SwerveDrivetrainConstants m_swerveDriveConstants;

    // --- Physical Properties --- 
    // Radius of the modules wheel in meters
    private static final double kWheelBaseLengthM = Units.inchesToMeters(22.75);
    private static final double kWheelTrackWidthM = Units.inchesToMeters(20.75);

    // --- Theoretical Properties ---
    // Revolutions per second * PI * Wheel Radius(m) / drive gear ratio
    private static final double kMaxRobotSpeedMeterPerSecond = (Swerve.kDriveMotorFreeSpeedRPM / 60.0) * Math.PI * Swerve.kWheelRadiusM / Swerve.kModuleDriveGearRatio;

    // ---  CAN Bus Config  --- 
    // (Note IDs can be the same for different device types (TalonFX, Pigeon2, etc), ie gyro and front left drive motor have the same ID)
    private static final String kCanivoreBusName = "CANivore";

    // --- Misc. Config ---
    private static final String kRobotName = "Spectre";
    
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
        private static final CANDeviceID kID = new CANDeviceID(1, kCanivoreBusName);

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
            )
            ;

    }
    
    // --- Swerve Modules Config ---
    private class Swerve {
        private static final double kDriveFrictionVoltage = 0.25; // The minimum voltage required for the drive motor to begin moving
        private static final double kSteerFrictionVoltage = 0.001; // The minimum voltage required for the steer motor to begin moving

        private static final double kDriveInertia = 0.001; // The rotational inertia in the drive system (kg * m^2) and represent rotational resistance to acceleration
        private static final double kSteerInertia = 0.00001; // The rotational inertia in the drive system (kg * m^2) and represent rotational resistance to acceleration

        // Radius of the modules wheel in meters
        private static final double kWheelRadiusM = Units.inchesToMeters(1.95);

        private static final double kDriveMotorFreeSpeedRPM  = 6000; // 6000 RPM is a Kraken X60

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

        // Swerve module encoder offsets
        // Offset from what the encoder thinks is 0 to the true zero of the module in ROTATIONS
        private static final double kFrontLeftEncoderOffsetRotations = -0.30517578125 + 0.5;
        private static final double kFrontRightEncoderOffsetRotations = -0.008544921875;
        private static final double kBackLeftEncoderOffsetRotations = -0.341064453125 + 0.5;
        private static final double kBackRightEncoderOffsetRotations = 0.100830078125 - 0.5;

        // Motor Configurations

        // Drive Motor
        private static final double kDriveMotorSupplyCurrentLimit = 50.0; // The amount of current (amps) that this motor is allowed to pull from the battery, if exceeded voltage will be reduced to avoid brownouts
        private static final double kDriveMotorStatorCurrentLimit = 100.0;  // The amount of current (amps) that motor is allowed to draw up to
        private static final double kDriveMotorSlipCurrent = 120; // The amount of current (amps) that can be applied to the drive wheel before it slips (120 basically means it doesn't slip)
        private static final TalonFXConfiguration kDriveMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(kDriveMotorSupplyCurrentLimit)
                    .withStatorCurrentLimit(kDriveMotorStatorCurrentLimit)
            );

        // Steer Motor
        private static final double kSteerMotorSupplyCurrentLimit = 30.0; // amps
        private static final double kSteerMotorStatorCurrentLimit = 90.0;  // amps
        private static final TalonFXConfiguration kSteerMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(kSteerMotorSupplyCurrentLimit)
                    .withStatorCurrentLimit(kSteerMotorStatorCurrentLimit)
            );

        // Steer Encoder
        private static final CANcoderConfiguration kSteerEncoderConfiguration = new CANcoderConfiguration();

        // Gear Reductions 
        // Stage 1
        private static final double kModuleDriveMotorPinionTeeth = 12.0;     // Number of pinions on the motor output shaft
        private static final double kModuleFirstGearTeeth = 54.0;            // Number of teeth on the gear the pinion is connected to

        // Stage 2
        private static final double kModuleStage2FirstGearTeeth = 32.0;      // Number of pinions on the motor output shaft
        private static final double kModuleStage2SecondGearTeeth = 25.0;     // Number of teeth on the gear the pinion is connected to

        // Stage 3 (output)
        private static final double kModuleBevelPinionTeeth = 15.0;          // Number of pinions on the motor output shaft
        private static final double kModuleBevelGearTeeth = 30.0;            // Number of teeth on the gear the pinion is connected to

        // Overall drive gear ratio from motor to wheel
        private static final double kModuleDriveGearRatio = 
            (kModuleFirstGearTeeth / kModuleDriveMotorPinionTeeth) *
            (kModuleStage2SecondGearTeeth / kModuleStage2FirstGearTeeth) * 
            (kModuleBevelGearTeeth / kModuleBevelPinionTeeth);

        // Overall steer great ratio from motor to steer output shaft 
        private static final double kModuleSteerGearRatio = 301.0 / 9.0;     // Unfortunately the 2910 code doesn't have the steer gear ratio broken down

        /**
         * The coupled gear ratio between the CanCoder and the drive motor.
         * Every 1 rotation of the steer motor results in coupled ratio of drive turns.
         */
        private static final double kModuleCouplingGearRatio = kModuleFirstGearTeeth / kModuleDriveMotorPinionTeeth;

        // Front Left
        private static final CANDeviceID kFrontLeftDriveMotor = new CANDeviceID(1, kCanivoreBusName);
        private static final CANDeviceID kFrontLeftSteerMotor = new CANDeviceID(2, kCanivoreBusName);
        private static final CANDeviceID kFrontLeftSteerEncoder = new CANDeviceID(1, kCanivoreBusName);

        // Front Right
        private static final CANDeviceID kFrontRightDriveMotor = new CANDeviceID(3, kCanivoreBusName);
        private static final CANDeviceID kFrontRightSteerMotor = new CANDeviceID(4, kCanivoreBusName);
        private static final CANDeviceID kFrontRightSteerEncoder = new CANDeviceID(2, kCanivoreBusName);

        // Back Left
        private static final CANDeviceID kBackLeftDriveMotor = new CANDeviceID(5, kCanivoreBusName);
        private static final CANDeviceID kBackLeftSteerMotor = new CANDeviceID(6, kCanivoreBusName);
        private static final CANDeviceID kBackLeftSteerEncoder = new CANDeviceID(3, kCanivoreBusName);

        // Back Right
        private static final CANDeviceID kBackRightDriveMotor = new CANDeviceID(7, kCanivoreBusName);
        private static final CANDeviceID kBackRightSteerMotor = new CANDeviceID(8, kCanivoreBusName);
        private static final CANDeviceID kBackRightSteerEncoder = new CANDeviceID(4, kCanivoreBusName);

        /**
         * Class used to categories different constants within the config
         */
        private class Modules {

            // Front Left Module
            private static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kFrontLeftSwerveModuleConstants = Swerve.Modules.build(
                Swerve.kFrontLeftDriveMotor, 
                Swerve.kFrontLeftSteerMotor, 
                Swerve.kFrontLeftSteerEncoder, 
                kWheelBaseLengthM / 2, 
                kWheelTrackWidthM / 2
            );
            
            // Front Right Module
            private static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kFrontRightSwerveModuleConstants = Swerve.Modules.build(
                Swerve.kFrontRightDriveMotor, 
                Swerve.kFrontRightSteerMotor, 
                Swerve.kFrontRightSteerEncoder, 
                kWheelBaseLengthM / 2, 
                -kWheelTrackWidthM / 2
            );

            // Back Left Swerve Module
            private static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kBackLeftSwerveModuleConstants = Swerve.Modules.build(
                Swerve.kBackLeftDriveMotor, 
                Swerve.kBackLeftSteerMotor, 
                Swerve.kBackLeftSteerEncoder, 
                -kWheelBaseLengthM / 2, 
                kWheelTrackWidthM / 2
            );

            // Back Right Swerve Module
            private static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kBackRightSwerveModuleConstants = Swerve.Modules.build(
                Swerve.kBackRightDriveMotor, 
                Swerve.kBackRightSteerMotor, 
                Swerve.kBackRightSteerEncoder, 
                -kWheelBaseLengthM / 2, 
                -kWheelTrackWidthM / 2
            );

            /**
             * Build a single swerve module with the required information
             * @param driveMotorID CANDeviceID used to store this swerve modules drive motor ID
             * @param steerMotorID CANDeviceID used to store this swerve modules steer motor ID
             * @param steerEncoderID CANDeviceID used to store this swerve modules steer encoder ID
             * @param xLocationMeters Offset in meters along the X (wheel length) relative to the center of the robot as to where this swerve module is located
             * @param yLocationMeters Offset in meters along the Y (wheel base) relative to the center of the robot as to where this swerve module is located
             * @return New swerve module constants object constructed with the desired data
             */
            private static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> build(
                CANDeviceID driveMotorID,CANDeviceID steerMotorID,
                CANDeviceID steerEncoderID,
                double xLocationMeters,
                double yLocationMeters
            ){
                return new SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorId(driveMotorID.getDeviceID())
                .withSteerMotorId(steerMotorID.getDeviceID())
                .withEncoderId(steerEncoderID.getDeviceID())
                .withDriveMotorGearRatio(Swerve.kModuleDriveGearRatio)
                .withSteerMotorGearRatio(Swerve.kModuleSteerGearRatio)
                .withCouplingGearRatio(Swerve.kModuleCouplingGearRatio)
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(false)
                .withEncoderInverted(false)
                .withEncoderOffset(Swerve.kFrontRightEncoderOffsetRotations)
                .withLocationX(xLocationMeters)
                .withLocationY(yLocationMeters)
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(Swerve.kDriveMotorGains)
                .withSteerMotorGains(Swerve.kSteerMotorGains)
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(Swerve.kDriveMotorConfiguration)
                .withSteerMotorInitialConfigs(Swerve.kSteerMotorConfiguration)
                .withEncoderInitialConfigs(Swerve.kSteerEncoderConfiguration)
                .withDriveFrictionVoltage(Swerve.kDriveFrictionVoltage)
                .withSteerFrictionVoltage(Swerve.kSteerFrictionVoltage)
                .withDriveInertia(kDriveInertia)
                .withSteerInertia(kSteerInertia)
                .withSlipCurrent(kDriveMotorSlipCurrent)
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
                .withSpeedAt12Volts(kMaxRobotSpeedMeterPerSecond)
                .withWheelRadius(kWheelRadiusM);
            }
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
        // Configure the ports for this bot
        m_portConfiguration = buildPortConfiguration();

        // Configure the cameras on this bot
        m_cameraConfigurations = buildCameraConfigurations();

        // Construct our swerve module representations
        m_moduleConstants = buildSwerveModuleConstants();

        // Construct our constants for the overall swerve drive
        m_swerveDriveConstants = buildSwerveDriveConstants();
    }

    /**
     * Setup the port configuration for this bot
     * @return A configured PortConfiguration for this bot
     */
    private PortConfiguration buildPortConfiguration(){
        return new PortConfiguration()
            .withCANBusName(kCanivoreBusName);
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
            Swerve.Modules.kFrontLeftSwerveModuleConstants,
            Swerve.Modules.kFrontRightSwerveModuleConstants,
            Swerve.Modules.kBackLeftSwerveModuleConstants,
            Swerve.Modules.kBackRightSwerveModuleConstants
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

    // --- RobotConfig override functions ---
    @Override
    public SwerveDrivetrainConstants getSwerveDriveConstants() {
        return m_swerveDriveConstants;
    }

    @Override
    public List<SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>> getModuleConstants() {
        return m_moduleConstants;
    }

    @Override
    public PortConfiguration getPortConfiguration() {
        return m_portConfiguration;
    }

    @Override
    public String getRobotName() {
        return kRobotName;
    }

    @Override
    public List<CameraConfiguration> getCameraConfigurations() {
        return m_cameraConfigurations;
    }
}
