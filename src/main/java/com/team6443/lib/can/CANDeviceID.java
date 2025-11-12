// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.can;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

/**
 * Abstraction layer for representing some device that uses CAN to communicate and has both a CAN ID and a CAN BUS
 * 
 * Largely yoinked from 2910 (Jack in the Bot): https://github.com/FRCTeam2910/2025CompetitionRobot-Public/blob/main/src/main/java/org/frc2910/robot/util/CanDeviceId.java
 * Added CAN status checking functionality in addition
 */
public class CANDeviceID {
    /**
     * There are only so many possible devices we can have connected to so we can use an enum
    */
    public enum CANDeviceType {
        TALON_FX("TalonFX"),
        CANCODER("CANcoder"),
        PIGEON2("Pigeon2")
        ;

        private final String printableName;
        private CANDeviceType(String name){
            this.printableName = name;
        }

        @Override
        public String toString() {
            return this.printableName;
        }
    }

    private final String deviceName;
    private final int canID;
    private final String busName;
    private final CANDeviceType deviceType;
    private final String subsystemName;

    // Set when this CAN device is in use by a CTRE device
    private StatusSignal<?> ctreStatusSignal = null;

    // If we are following another CAN device this will be set
    private CANDeviceID masterDevice = null;
    
    /**
     * Create a new CAN Device that is used to represent any number of CAN devices on the network
     * @param canID ID of this CAN device
     * @param deviceName Name of this CAN device
     * @param subsystemName Name of the subsystem that this CAN device is a part of
     * @param deviceType Type of CAN device that this object is
     * @param busName The bus this device exists on
     */
    public CANDeviceID(
            int canID, 
            String deviceName,
            String subsystemName,
            CANDeviceType deviceType, 
            String busName
        ){
        this.deviceName = deviceName;
        this.canID = canID;
        this.busName = busName;
        this.deviceType = deviceType;
        this.subsystemName = subsystemName;
    }

    // Use the default bus name "rio".
    public CANDeviceID(
        int canID, 
        String deviceName, 
        String subsystemName,
        CANDeviceType deviceType
    ) {
        this(canID, deviceName, subsystemName, deviceType, "rio");
    }

    public int getDeviceID() {
        return canID;
    }

    public String getBus() {
        return busName;
    }

    public CANDeviceType getDeviceType() {
        return deviceType;
    }

    public CANDeviceID getMasterDevice(){
        return this.masterDevice;
    }
    public void setMasterCANDevice(CANDeviceID master){
        this.masterDevice = master;
    }

    /**
     * CTRE Handler for setting status signal to check if a CAN device is connected or not
     * @param signal The signal we are checking to verify can state
     */
    public void setStatusSignal(StatusSignal<?> signal){
        ctreStatusSignal = signal;
    }

    /**
     * CTRE Handler for setting status signal to check if a CAN device is connected or not
     * @param signal The signal we are checking to verify can state
     * @param hz Frequency at which the signal should be updated in Hz
     */
    public void setStatusSignal(StatusSignal<?> signal, double hz){
        ctreStatusSignal = signal;
        if(this.ctreStatusSignal != null){
            this.ctreStatusSignal.setUpdateFrequency(hz);
        }
    }

    /**
     * Reference to the CTRE status signal for this device
     * @return The status signal that this device is connected to
     */
    public StatusSignal<?> getCTREStatusSignal(){
        return ctreStatusSignal;
    }

    public boolean isConnected(){
        if(ctreStatusSignal != null){
            return ctreStatusSignal.getStatus() == StatusCode.OK;
        }

        // Need to implement other possible way to handle non CTRE CAN devices maybe
        return false;
    }

    public boolean equals(CANDeviceID other) {
        return other.canID == canID && other.busName.equals(busName) && other.deviceType == deviceType;
    }

    /**
     * Returns a string consisting of GivenName_ID
     * @return String created from device information
     */
    public String getDeviceName(){
        return deviceName + "_" + canID;
    }

    /**
     * Get the name of the parent subsystem of which this CAN device is a part of
     * @return Name of top level subsystem
     */
    public String getSubsystemName(){
        return subsystemName;
    }

    @Override
    public String toString() {
        // GivenName_TYPE_BUS
        return getDeviceName() + "_" + busName;
    }
}
