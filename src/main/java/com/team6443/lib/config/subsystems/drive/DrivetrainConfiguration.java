// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.subsystems.drive;

import java.util.List;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.team6443.lib.can.CANDeviceID;
import com.team6443.lib.config.odometry.OdometryStandardDevs;

/** 
 * Defines how a given drive subsystem is setup
 */
public class DrivetrainConfiguration {
    public String kConfigurationName;
    public double kChassisTranslationSpeedThreshold; // Anything less than this chassis speed in meters per second will be set to 0
    public double kChassisRotationalSpeedThreshold; // Anything less than this chassis speed in radians per second will be set to 0

    public double kSteerJoystickDeadband;
    public double kDriveJoystickDeadband;

    public double kMaxDriveSpeed;
    public double kMaxAngularRate;

    public CANDeviceID kGyroDeviceID;
    public SwerveModuleConstants<?, ?, ?>[] kModuleConstants;
    public SwerveDrivetrainConstants kDriveConstants;

    public OdometryStandardDevs kEnabledOdometryStandardDevs;
    public OdometryStandardDevs kDisabledOdometryStandardDevs;

    public DrivetrainConfiguration(){}

    public DrivetrainConfiguration withName(String name){
        this.kConfigurationName = name;
        return this;
    }

    public DrivetrainConfiguration withGyroDevice(CANDeviceID device){
        this.kGyroDeviceID = device;
        return this;
    }

    public DrivetrainConfiguration withChassisSpeedDeadband(double minTranslationMS, double minRotationRadS){
        this.kChassisTranslationSpeedThreshold = minTranslationMS;
        this.kChassisRotationalSpeedThreshold = minRotationRadS;
        return this;
    }

    public DrivetrainConfiguration withModuleConstants(SwerveModuleConstants<?, ?, ?>[] moduleConstants){
        this.kModuleConstants =  moduleConstants;
        return this;
    }

    public DrivetrainConfiguration withDrivetrainConstants(SwerveDrivetrainConstants constants){
        this.kDriveConstants = constants;
        return this;
    }

    public DrivetrainConfiguration withMaxDriveSpeed(double speed){
        this.kMaxDriveSpeed = speed;
        return this;
    }

    public DrivetrainConfiguration withMaxAngularRate(double rate){
        this.kMaxAngularRate = rate;
        return this;
    }

    public DrivetrainConfiguration withJoystickDeadband(double steerDeadband, double driveDeadband){
        this.kSteerJoystickDeadband = steerDeadband;
        this.kDriveJoystickDeadband = driveDeadband;
        return this;
    }

    public DrivetrainConfiguration withOdometryStandardDevs(OdometryStandardDevs enabledStandardDevs, OdometryStandardDevs disabledStandardDevs){
        this.kEnabledOdometryStandardDevs = enabledStandardDevs;
        this.kDisabledOdometryStandardDevs = disabledStandardDevs;
        return this;
    }
}
