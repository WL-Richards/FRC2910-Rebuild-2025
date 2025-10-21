// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.config;

import java.util.Map;

import com.team6443.lib.network.NetworkUtils;

/**
 * Enum (basically a class here) to facilitate the identification and proper configuration selection between different robots
 */
public enum RobotID {
    NAUTILUS,
    ;   
    // ---After this point we can define class properties in the enum ---

    /* Define mappings of robot types to the MAC address they are associated with */
    private static final Map<String, RobotID> RobotToMAC = Map.of(
        "blah:blah:blah:blah:blah:blah", RobotID.NAUTILUS
    );

    /**
     * Static method used to ascertain the current robot that this code is being run on by comparing the HW MAC address to a known one
     */
    public static RobotID getIdentification(){
        String macAddress = NetworkUtils.MAC.getMACAddress();
        if (macAddress == null){
            return RobotID.NAUTILUS;
        }
        RobotID id = RobotToMAC.get(macAddress);

        // Default to the main robot if MAC was unable to be retrieved
        return  (id != null) ? id : RobotID.NAUTILUS;
    }
}
