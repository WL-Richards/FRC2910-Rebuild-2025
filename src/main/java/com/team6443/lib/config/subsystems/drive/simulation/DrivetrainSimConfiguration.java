// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.subsystems.drive.simulation;

import com.team6443.lib.config.robot.PhysicalConfiguration;

/** Configuration for how this drive train should be simulated */
public class DrivetrainSimConfiguration {
    
    /* Rate in milliseconds that the sim will loop */
    public String kConfigurationName;

    /* Rate in milliseconds that the sim will loop */
    public final double kSimLoopPeriodMS; 

    /* Describes physical properties about the robot: weight, track width + length, wheel coef. of friction, etc. */
    public PhysicalConfiguration kPhysicalConfiguration;

    /* Number of motors being used to drive and steer a given swerve module */
    public int kModuleDriveMotorCount;
    public int kModuleSteerMotorCount;

    public DrivetrainSimConfiguration(double loopPeriod){
        this.kSimLoopPeriodMS = loopPeriod;
    }

    public DrivetrainSimConfiguration withPhysicalConfiguration(PhysicalConfiguration config){
        this.kPhysicalConfiguration = config;
        return this;
    }
    
    public DrivetrainSimConfiguration withModuleDriveMotorCount(int driveMotorCount) {
        this.kModuleDriveMotorCount = driveMotorCount;
        return this;
    }
    
    public DrivetrainSimConfiguration withModuleSteerMotorCount(int steerMotorCount) {
        this.kModuleSteerMotorCount = steerMotorCount;
        return this;
    }

    public DrivetrainSimConfiguration withName(String name) {
        this.kConfigurationName = name;
        return this;
    }
}
