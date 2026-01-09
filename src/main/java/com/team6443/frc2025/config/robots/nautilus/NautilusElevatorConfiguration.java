// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.config.robots.nautilus;

import java.util.List;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team6443.lib.config.motors.MotorFollowerConfiguration;
import com.team6443.lib.config.motors.factories.TalonFXConfigurationFactory;
import com.team6443.lib.config.subsystems.elevator.simulation.SimulatedElevatorConfiguration;
import com.team6443.lib.config.wrappers.ConfigureSlot0Gains;
import com.team6443.lib.core.can.CANDeviceID;
import com.team6443.lib.core.motors.interfaces.MotorIO.FollowDirection;

import edu.wpi.first.math.util.Units;

/** 
 * Elevator Configuration for nautilus
 */
public class NautilusElevatorConfiguration {

    // The max height the elevator can extend to in meters
    public final double kMaxHeight = 1.15;

    // The min height the elevator can retract to in meters
    public final double kMinHeight = 0;

    // The max velocity that we want the elevator to be capable of m/s
    public final double kMaxVelocity = 3.6;

    // The max acceleration that we want the elevator to be capable of m/s²
    public final double kMaxAcceleration = 4.0;

    // The max acceleration that we want the elevator to be capable of m/s³
    public final double kJerk = 0.0;

    // ID of the the leader elevator motor
    public final int kTopLeaderMotorID = 13;
    public final double kTopLeaderMotorCurrentLimit = 50.0;

    // ID of the the follower elevator motor
    public final int kBottomFollowerMotorID = 12;
    public final double kBottomFollowerMotorCurrentLimit = 50.0;

    // Defines the raidus of the pulley/drum that the elevator winch cable is wrapped around
    public final double kElevatorDrumRadius = Units.inchesToMeters(1.128);

    // Gear ratio between the motor and the elevator drum
    public final double kMotorToDrumGearing = (11.0 / 50.0);

    // Compute the ratio between the rotor of the motor turning and the output units of the system
    public final double kElevatorUnitToRotorRatio = kMotorToDrumGearing * 2.0 * kElevatorDrumRadius * Math.PI;

    // Motion magic gains for the elevator
    public final ConfigureSlot0Gains kMotorGains = new ConfigureSlot0Gains(
        1.0,
        0.0,
        0.0,
        0.325,
        0.075,
        0.1333,
        0.005
        )
    .withGravityType(GravityTypeValue.Elevator_Static);

    public final SimulatedElevatorConfiguration kSimulatedElevatorConfiguration;
    public final MotorFollowerConfiguration<TalonFXConfiguration> kElevatorMotorSubsystemConfiguration;
    
    public NautilusElevatorConfiguration(
        String elevatorSubsystemName, 
        String elevatorSubsystemCANBus
    ){

        // ------------------------- Configure Simulated elevator constants -------------------------
        kSimulatedElevatorConfiguration = new SimulatedElevatorConfiguration();
        kSimulatedElevatorConfiguration.carriageMass = 1.97312681;
        kSimulatedElevatorConfiguration.drumRadius = kElevatorDrumRadius;
        kSimulatedElevatorConfiguration.gearing = kMotorToDrumGearing;
        kSimulatedElevatorConfiguration.meterToRotorRatio = kElevatorUnitToRotorRatio;


        // ------------------------- Elevator Follower Motor Configuration----------------------------
        MotorFollowerConfiguration.FollowerConfiguration<TalonFXConfiguration> followerConfig = TalonFXConfigurationFactory.generateFollowerTalonFXConfiguration();

        followerConfig.config.kConfigurationName = "BottomMotorFollower";
        followerConfig.config.kCANDevice = new CANDeviceID(
            kBottomFollowerMotorID, 
            "BottomMotorFollower",  
            elevatorSubsystemName,
            CANDeviceID.CANDeviceType.TALON_FX, 
            elevatorSubsystemCANBus
        );
        followerConfig.config.kUnitToRotorRotationRatio = kElevatorUnitToRotorRatio;

        // Setup current limit on follower
        followerConfig.config.kMotorConfig.CurrentLimits.StatorCurrentLimit = kBottomFollowerMotorCurrentLimit;
        followerConfig.config.kMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Set follow direction and motor NeutralMode
        followerConfig.followDirection = FollowDirection.SAME;
        followerConfig.config.kMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        // ------------------------- Elevator Subsystem Configuration -------------------------
        kElevatorMotorSubsystemConfiguration = 
        new MotorFollowerConfiguration<>(
            new TalonFXConfiguration()
        );

        kElevatorMotorSubsystemConfiguration.kConfigurationName = "ElevatorSubsystem";
        kElevatorMotorSubsystemConfiguration.kCANDevice = new CANDeviceID(
            kTopLeaderMotorID, 
            "TopMotorLeader",  
            elevatorSubsystemName,
            CANDeviceID.CANDeviceType.TALON_FX, 
            elevatorSubsystemCANBus
        );

        // Configure Elevator motor gains
        kElevatorMotorSubsystemConfiguration.kMotorConfig.Slot0 = kMotorGains;

        // Configure elevator rotor ratio
        kElevatorMotorSubsystemConfiguration.kUnitToRotorRotationRatio = kElevatorUnitToRotorRatio;

        // Configure elevator motion magic parameters
        kElevatorMotorSubsystemConfiguration.kMotorConfig.MotionMagic.MotionMagicCruiseVelocity = kMaxVelocity / kElevatorMotorSubsystemConfiguration.kUnitToRotorRotationRatio;
        kElevatorMotorSubsystemConfiguration.kMotorConfig.MotionMagic.MotionMagicAcceleration = kMaxAcceleration / kElevatorMotorSubsystemConfiguration.kUnitToRotorRotationRatio;
        kElevatorMotorSubsystemConfiguration.kMotorConfig.MotionMagic.MotionMagicJerk = kJerk / kElevatorMotorSubsystemConfiguration.kUnitToRotorRotationRatio;

        // Configure motor in brake mode
        kElevatorMotorSubsystemConfiguration.kMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Enable stator current limiting
        kElevatorMotorSubsystemConfiguration.kMotorConfig.CurrentLimits.StatorCurrentLimit = kTopLeaderMotorCurrentLimit;
        kElevatorMotorSubsystemConfiguration.kMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Configure max and min positions
        kElevatorMotorSubsystemConfiguration.kMaxPositionUnits = kMaxHeight;
        kElevatorMotorSubsystemConfiguration.kMinPositionUnits = kMinHeight;

        // Follower configurations 
        kElevatorMotorSubsystemConfiguration.followerConfigurations = List.of(followerConfig);
        
    }
}
