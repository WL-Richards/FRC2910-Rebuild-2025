// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6443.lib.can.CANDeviceID;

/** 
 * Configuration for treating a motor as effectively a servo
 * 
 * This allows for specifiying a unit conversion rate and min and max limits 
 */
public class TalonFXServoMotorConfiguration extends ServoMotorConfiguration {

    // TalonFX to use with the motor
    public TalonFXConfiguration config = null;

    public TalonFXServoMotorConfiguration() {}

    /**
     * Set a new configuration to be used here 
     * @param config we want to use instead
     * @return Reference to this configuration
     */
    public TalonFXServoMotorConfiguration withConfig(TalonFXConfiguration config){
        this.config = config;
        return this;
    }
}
