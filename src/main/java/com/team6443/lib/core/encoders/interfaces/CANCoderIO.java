// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.core.encoders.interfaces;

import com.team6443.lib.core.encoders.CANCoderInputs;

/** 
 * Generic interface for CAN coder
 */
public interface CANCoderIO {
    /**
     * Get then name of the motor to use
     * @return Get the name given to this motor
     */
    public String getName();

    // ------ Encoder outputs ------
    /**
     * Update the MotorInputs object passed in with the current inputs to the motor
     * @param inputs The inputs that are currently applied to the motor
     * @return true on success false on failure
     */
    public boolean updateInputs(CANCoderInputs inputs);

    /**
     * Set the update rate of this CAN coder in Hz
     * @param hz Rate at which to update the state of this CAN coder in Hz
     * @return true on success false on failure
     */
    public boolean setUpdateFrequency(double hz);
    
} 
