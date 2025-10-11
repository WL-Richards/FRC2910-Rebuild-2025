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
public class TalonFXServoMotorConfiguration {

    // The name of the configuration in use
    public String ConfigurationName = null;

    // The CAN device that is used by the Talon being configured
    public CANDeviceID CANDevice = null;

    // TalonFX to use with the motor
    public TalonFXConfiguration config = null;

    // Conversion factor for converting between the desired output units and rotations of the motor
    public double unitToRotorRotationRatio = 1.0;

    // In whatever units this configruation is using set the minimum position that this motor can drive to
    public double kMinPositionUnits = Double.NEGATIVE_INFINITY;

    // In whatever units this configuration is using set the maximum position that this motor can drive to
    public double kMaxPositionUnits = Double.POSITIVE_INFINITY;

    // Moment of Inertia (KgMetersSquared) (how resistant a motor's rotor is to changs in its rotational speed)
    public double momentOfInertia = 0.5;

    public TalonFXServoMotorConfiguration() {}

    /**
     * Setup the servo motor configuration to use the desired CAN device
     * @param device CAN device we want to use with this configuration
     * @return Reference to this configuration
     */
    public TalonFXServoMotorConfiguration withCANDevice(CANDeviceID device){
        this.CANDevice = device;
        return this;
    }

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
