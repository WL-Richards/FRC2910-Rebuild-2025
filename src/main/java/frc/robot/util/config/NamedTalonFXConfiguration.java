// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.util.can.CANDeviceID;

/** Small extension of the TalonFX configuration to associate a string name and CAN device ID with its */
public class NamedTalonFXConfiguration extends TalonFXConfiguration {
    public final String DeviceName;
    public final CANDeviceID DeviceID;

    public NamedTalonFXConfiguration(String name, CANDeviceID canDevice){
        this.DeviceName = name;
        this.DeviceID = canDevice;
    }

    
}
