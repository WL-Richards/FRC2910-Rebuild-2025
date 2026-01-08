// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.core.encoders.factories;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6443.lib.config.encoders.CANCoderConfiguration;
import com.team6443.lib.core.can.CANDeviceID;
import com.team6443.lib.core.can.CANStatusLogger;
import com.team6443.lib.core.encoders.io.CANCoderHardwareIO;
import com.team6443.lib.core.phoenix6.CTREUtil;

import edu.wpi.first.wpilibj.RobotBase;

/** 
 * Static factory class for creating instances of our CANCoder device
 */
public class CANCoderFactory {

    /**
     * Create a new CANCoderIOHardware instance with the specified configuration, this will automatically select between Sim or real depending on bot state
     * @param config CANCoderConfiguration to use to setup this can coder
     * @return
     */
    public static CANCoderHardwareIO createIO(CANCoderConfiguration config){
        // TODO: UPDATE TO ACTUALLY DO SOMETHING DIFF IF IN SIM
        return new CANCoderHardwareIO(config);
    }

    /**
     * Create a new raw CANcoder with the CANcoderConfiguration supplied
     * @param device CANDevice the represents the CANcoder being created
     * @param talonConfig The config that should be applied to the CANcoder after its created
     * @return The newly created CANcoder instance
     */
    public static CANcoder createRawWithConfig(CANDeviceID device, CANcoderConfiguration config){
        CANcoder encoder = createRaw(device);
        
        CTREUtil.Configuration.Encoders.applyConfiguration(encoder, config);

        // Set update rate of our CANDeviceID status signal to update at 100 hz
        device.setStatusSignal(encoder.getSupplyVoltage(), 100);

        // Automatically register the Talon with the CAN status logger upon creation 
        CANStatusLogger.get(device.getBus()).registerCANDevice(device);
        return encoder;
    }

    /**
     * Create a new raw CANcoder
     * @param device CAN device that represents this CANcoder
     * @return The newly created CANcoder
     */
    private static CANcoder createRaw(CANDeviceID device) {
        CANcoder encoder = new CANcoder(device.getDeviceID(), device.getBus());
        encoder.clearStickyFaults();
        return encoder;
    }

}
