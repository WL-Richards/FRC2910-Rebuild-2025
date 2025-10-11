// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.can.interfaces;

import com.team6443.lib.can.CANDeviceID;

/** 
 * Interface to specify whether or not a class has a CANDeviceID associated with it
 * 
 * (no pun intended)
*/
public interface CANable {

    /**
     * Retrieve the CAN device associated with this object
     * @return CAN Device that this object is represented by
     */
    public CANDeviceID getCANDevice();

    /**
     * Get the CAN ID for the associated CAN device
     * @return ID of the CAN device
     */
    public default int getCANDeviceID(){
        return getCANDevice().getDeviceID();
    }

    /**
     * Get the CAN Device ID name that we have set
     * @return The name of the CAN device
     */
    public default String getCANDeviceName(){
        return getCANDevice().getDeviceName();
    }
}
