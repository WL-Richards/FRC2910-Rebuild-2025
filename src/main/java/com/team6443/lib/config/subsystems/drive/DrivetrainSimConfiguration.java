// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.subsystems.drive;


/** Configuration for how this drive train should be simulated */
public class DrivetrainSimConfiguration {
    public String kConfigurationName;

    public final double kSimLoopPeriodMS; // Rate in milliseconds that the sim will loop

    public double kRobotWeightPounds;

    public double kBumperWidthInches;
    public double kBumperLengthInches;

    public int kDriveMotorCount;
    public int kSteerMotorCount;

    public double kWheelCoefficientOfFriction;

    public DrivetrainSimConfiguration(double loopPeriod){
        this.kSimLoopPeriodMS = loopPeriod;
    }

    public DrivetrainSimConfiguration withRobotWeightPounds(double weightPounds) {
        this.kRobotWeightPounds = weightPounds;
        return this;
    }
    
    public DrivetrainSimConfiguration withBumperWidthInches(double bumperWidthInches) {
        this.kBumperWidthInches = bumperWidthInches;
        return this;
    }
    
    public DrivetrainSimConfiguration withBumperLengthInches(double bumperLengthInches) {
        this.kBumperLengthInches = bumperLengthInches;
        return this;
    }
    
    public DrivetrainSimConfiguration withDriveMotorCount(int driveMotorCount) {
        this.kDriveMotorCount = driveMotorCount;
        return this;
    }
    
    public DrivetrainSimConfiguration withSteerMotorCount(int steerMotorCount) {
        this.kSteerMotorCount = steerMotorCount;
        return this;
    }
    
    public DrivetrainSimConfiguration withWheelCoefficientOfFriction(double coefficientOfFriction) {
        this.kWheelCoefficientOfFriction = coefficientOfFriction;
        return this;
    }

    public DrivetrainSimConfiguration withName(String name) {
        this.kConfigurationName = name;
        return this;
    }
}
