// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.motors.factories;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6443.lib.config.motors.ServoMotorConfiguration;
import com.team6443.lib.config.motors.ServoMotorFollowerConfiguration;

/** 
 * Factory for creating Talon FX configs
 */
public class TalonFXConfigurationFactory {
    public static ServoMotorFollowerConfiguration.FollowerConfiguration<TalonFXConfiguration> generateFollowerTalonFXConfiguration(){
        return new ServoMotorFollowerConfiguration.FollowerConfiguration<>(
                    new ServoMotorConfiguration<TalonFXConfiguration>()
                    .withConfig(new TalonFXConfiguration())
                );
    }
}
