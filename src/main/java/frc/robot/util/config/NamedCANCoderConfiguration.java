// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

import frc.robot.util.can.CANDeviceID;

/** Small extension of CANCoder configuration to add a device name and CAN Device ID */
public class NamedCANCoderConfiguration extends CANcoderConfiguration {
    public final String DeviceName;
    public final CANDeviceID DeviceID;

    public NamedCANCoderConfiguration(String name, CANDeviceID canDevice){
        this.DeviceName = name;
        this.DeviceID = canDevice;
    }
}
