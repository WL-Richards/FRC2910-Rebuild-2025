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

    private static final String kElevatorSubsystemName = "ElevatorSubsystem";
    private static final String kDriveSubsystemName = "DriveSubsystem";
    
    // --- Drivetrain Config ---
    private static final NautilusDriveConfiguration kDriveConfiguration = new NautilusDriveConfiguration(kDriveSubsystemName, kCanivoreBusName);
   
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

        // Defines the raidus of the pulley/drum that the elevator winch cable is wrapped around
        public static final double kElevatorDrumRadius = Units.inchesToMeters(1.128);

        // Gear ratio between the motor and the elevator drum
        public static final double kMotorToDrumGearing = (11.0 / 50.0);

        // Compute the ratio between the rotor of the motor turning and the output units of the system
        public static final double kElevatorUnitToRotorRatio = kMotorToDrumGearing * 2.0 * kElevatorDrumRadius * Math.PI;

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
        config.kMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Elevator.kMaxVelocity / config.kUnitToRotorRotationRatio;
        config.kMotorConfig.MotionMagic.MotionMagicAcceleration = Elevator.kMaxAcceleration / config.kUnitToRotorRotationRatio;
        config.kMotorConfig.MotionMagic.MotionMagicJerk = Elevator.kJerk / config.kUnitToRotorRotationRatio;

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
        config.gearing = Elevator.kMotorToDrumGearing;
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
