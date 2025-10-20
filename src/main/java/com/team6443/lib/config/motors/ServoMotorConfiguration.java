// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.motors;

import com.team6443.lib.can.CANDeviceID;

/** 
 * Configuration for treating a motor as effectively a servo
 * 
 * This allows for specifiying a unit conversion rate and min and max limits 
 * 
 * T in this case is the underlying configuration for the motor
 */
public class ServoMotorConfiguration<T> {

    // The actual configuration being used internally by the motor
    public T motorConfig = null;

    // The name of the configuration in use
    public String ConfigurationName = "UNNAMED";

    // The CAN device that is used by the device being configured
    public CANDeviceID CANDevice = null;

    // Conversion factor for converting between the desired output units and rotations of the motor
    public double unitToRotorRotationRatio = 1.0;

    // In whatever units this configruation is using set the minimum position that this motor can drive to
    public double kMinPositionUnits = Double.NEGATIVE_INFINITY;

    // In whatever units this configuration is using set the maximum position that this motor can drive to
    public double kMaxPositionUnits = Double.POSITIVE_INFINITY;

    // Moment of Inertia (KgMetersSquared) (how resistant a motor's rotor is to changs in its rotational speed)
    public double momentOfInertia = 0.5;

    /**
     * Convert the current rotor rotations to the real-world units specified scaled by the unitToRotorRotationRation defined in the config
     * @param rotorRotations Rotor rotation value we want to convert into the in-use units
     * @return The rotations converted into some units as defined in the config
     */
    public double getRotorRotationsToUnits(double rotorRotations){
        return rotorRotations * this.unitToRotorRotationRatio;
    }

    /**
     * Convert the current units into rotor rotations using the defined conversion ratio
     * @param units Units we want to convert to rotor rotations
     * @return The resulting rotor rotations 
     */
    public double getUnitsToRotorRotations(double units){
        return units / this.unitToRotorRotationRatio;
    }


    /**
     * Get whatever the specified motor specific configuration was
     * @return The motor configuration that this servo motor is using
     */
    public final T getMotorConfig(){
        return motorConfig;
    }

    /**
     * Update the configuration to some other config type
     * @param config What motor configuration should we use under the hood
     * @return Reference to this servo motor configuration for chaining
     */
    public ServoMotorConfiguration<T> withConfig(T config){
        this.motorConfig = config;
        return this;
    }
}
