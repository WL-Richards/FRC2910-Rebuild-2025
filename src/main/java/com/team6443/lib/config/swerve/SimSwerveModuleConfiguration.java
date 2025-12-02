// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.swerve;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.team6443.lib.motors.sim.TalonFXSimIO;
import com.team6443.lib.motors.sim.TalonFXWithCANcoderSimIO;

/** 
 * Handles basic swerve module configuration wrapper for simulated swerve modules when using MapleSim drivetrain
 */
public class SimSwerveModuleConfiguration {
    public final SwerveModuleConstants<?, ?, ?> moduleConstants;
    public final SwerveModuleSimulation moduleSimulation;       // Maple sim swerve module simulation component

    public SimSwerveModuleConfiguration(
            SwerveModuleConstants<?, ?, ?> moduleConstants,
            SwerveModuleSimulation moduleSimulation,
            SwerveModule<TalonFX, TalonFX, CANcoder> module,
            SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> moduleConfig
    ){
        this.moduleConstants = moduleConstants;

        // Setup module simulation
        this.moduleSimulation = moduleSimulation;

        // Configure drive motor controller to use a talon FX Sim IO
        this.moduleSimulation.useDriveMotorController(
            new TalonFXSimIO(
                moduleConfig.kDriveMotorID,
                module.getDriveMotor()
            )
        );

        // Configure steer motor controller to use a Talon FX wth Sim CAN coder
        this.moduleSimulation.useSteerMotorController(
            new TalonFXWithCANcoderSimIO(
                moduleConfig.kSteerMotorID,
                module.getSteerMotor(),
                moduleConfig.kSteerEncoderID,
                module.getEncoder()
            )
        );
        
    }
}
