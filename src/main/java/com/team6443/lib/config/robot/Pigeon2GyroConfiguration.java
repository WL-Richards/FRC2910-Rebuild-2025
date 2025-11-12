// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.team6443.lib.can.CANDeviceID;

import edu.wpi.first.math.geometry.Rotation3d;

/** 
 * Configuration for the Pigeon2 used on the robot
 */
public class Pigeon2GyroConfiguration {

    /**
     * The error of the gyro relative to what we determine is true 
     * 
     * To determine face robot in a direction that we know what the angle should be, GyroYawErrorDegrees = (gyro reading - true reading)
     */
    public double kGyroYawErrorDegrees = 0.0;

    // Rotation that the gyro is mounted all degrees passed in must be converted to radians
    public Rotation3d kMountRotation = null;

    // CAN Device ID associated with the gyro
    private CANDeviceID kCANDevice = null;

    // Configuration for the Pigeon2 that is to be used with the swerve drive
    private final Pigeon2Configuration kConfiguration = new Pigeon2Configuration();

    public Pigeon2GyroConfiguration() {
        withGyroYawError(kGyroYawErrorDegrees);
    }

    public Pigeon2GyroConfiguration withGyroYawError(double error){
        this.kGyroYawErrorDegrees = error;
        this.kConfiguration.withGyroTrim(
            new GyroTrimConfigs()
                .withGyroScalarZ(kGyroYawErrorDegrees)
        );
        return this;
    }

    public Pigeon2GyroConfiguration withGyroMountRotation(Rotation3d mountRot){
        this.kMountRotation = mountRot;
        this.kConfiguration.withMountPose(
            new MountPoseConfigs()
                .withMountPoseRoll(Math.toDegrees(kMountRotation.getX()))
                .withMountPosePitch(Math.toDegrees(kMountRotation.getY()))
                .withMountPoseYaw(Math.toDegrees(kMountRotation.getZ()))
        );
        return this;
    }

    public Pigeon2GyroConfiguration withCANDevice(CANDeviceID device){
        this.kCANDevice = device;
        return this;
    }
}
