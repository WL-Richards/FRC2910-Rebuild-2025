// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.encoders.hardware;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.team6443.lib.can.CANDeviceID;
import com.team6443.lib.can.interfaces.CANable;
import com.team6443.lib.config.encoders.CANCoderConfiguration;
import com.team6443.lib.encoders.CANCoderInputs;
import com.team6443.lib.encoders.interfaces.CANCoderIO;
import com.team6443.lib.factories.encoder.CANCoderFactory;
import com.team6443.lib.phoenix6.CTREUtil;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** 
 * Real hardware implementation of a CTRE CANcoder
*/
public class CANCoderIOHardware implements CANCoderIO, CANable {
    // Reference to the actual CANcoder that this object is wrapping
    protected final CANcoder encoder;

    // Defined configuration for how this CANcoder should be used
    protected CANCoderConfiguration config;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    
    private final BaseStatusSignal[] signals;

    private int warmupReadingCount = 0;
    private final int REQUIRED_WARMUP_CYCLES = 50;

    public CANCoderIOHardware(CANCoderConfiguration config){
        this.config = config;

        // Create a new instance of the CANcoder
        encoder = CANCoderFactory.createRawWithConfig(config.CANDevice, config.config);

        // Assign references to all the response signals for the CAN coder and set them to update at 100hz
        positionSignal = encoder.getAbsolutePosition();
        velocitySignal = encoder.getVelocity();
        signals = new BaseStatusSignal[] { positionSignal, velocitySignal };
        BaseStatusSignal.setUpdateFrequencyForAll(100, signals);
    }

    // ------ CANCoderIO Implementation ------
    @Override
    public String getName() {
        return getCANDeviceName();
    }
    
    @Override
    public boolean updateInputs(CANCoderInputs inputs) {
        BaseStatusSignal.refreshAll(signals);

        // Do sensor warmup for REQUIRED_WARMUP_CYCLES to ensure our readings are good before zeroing anything using them
        if(Double.isNaN(inputs.absolutePositionRotations))
        {
            BaseStatusSignal.waitForAll(10.0, signals);
            warmupReadingCount++;
            if(warmupReadingCount < REQUIRED_WARMUP_CYCLES){
                return false;
            }
        }
        
        // Sensor readings are updated
        inputs.absolutePositionRotations = positionSignal.getValue().in(Rotations);
        inputs.velocityRotationsPerSecond = velocitySignal.getValue().in(RotationsPerSecond);
        return true;
    }

    @Override
    public boolean setUpdateFrequency(double hz) {
        return BaseStatusSignal.setUpdateFrequencyForAll(hz, signals) == StatusCode.OK;
    }


    // ------ CANable Implementation ------
    @Override
    public CANDeviceID getCANDevice() {
        return this.config.CANDevice;
    }
    
    
}
