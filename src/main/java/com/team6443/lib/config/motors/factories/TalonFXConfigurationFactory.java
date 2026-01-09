// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.motors.factories;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6443.lib.config.motors.MotorConfiguration;
import com.team6443.lib.config.motors.MotorFollowerConfiguration;
import com.team6443.lib.core.can.CANDeviceID;

/** 
 * Factory for creating Talon FX configs
 */
public class TalonFXConfigurationFactory {
    public static MotorFollowerConfiguration.FollowerConfiguration<TalonFXConfiguration> generateFollowerTalonFXConfiguration(){
        return new MotorFollowerConfiguration.FollowerConfiguration<>(
                    new MotorConfiguration<TalonFXConfiguration>()
                    .withConfig(new TalonFXConfiguration())
                );
    }

    public static MotorFollowerConfiguration.FollowerConfiguration<TalonFXConfiguration> generateFollowerTalonFXConfiguration(String name, CANDeviceID device){
        return new MotorFollowerConfiguration.FollowerConfiguration<>(
                    new MotorConfiguration<TalonFXConfiguration>()
                        .withConfig(new TalonFXConfiguration())
                        .withName(name)
                        .withCANDevice(device)
                );
    }
}
