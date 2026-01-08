// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.encoders;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.team6443.lib.core.can.CANDeviceID;

/**
 * Configuration describing the setup of the CAN coder
 */
public class CANCoderConfiguration {

    // The CAN device that is used by the device being configured
    public CANDeviceID CANDevice = null;

    // The underlying CAN coder config that 
    public CANcoderConfiguration config = new CANcoderConfiguration();
}
