// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.can;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.team6443.frc2025.constants.RobotRuntimeConstants;
import com.team6443.lib.logging.interfaces.Loggable;

/** 
 * Class intended to managing logging of both the CAN bus itself and devices connected to the BUS
*/
public class CANStatusLogger implements Loggable {
    // ------ Singleton setup ------ 
    private static Map<String, CANStatusLogger> instances = new HashMap<String, CANStatusLogger>();

    /**
     * Retrieve the status logger for the desired CAN bus (supports multiple)
     * @return Reference to the desired CAN status logger
     */
    public static CANStatusLogger get(String busName){
        CANStatusLogger logger = instances.get(busName);
        if(logger == null){
            logger = new CANStatusLogger(busName);
            instances.put(busName, logger);
        }

        return logger;
    }

    /**
     * Retrieve the CANStatus logger by bus index
     * @return Reference to the desired CAN status logger
     */
    public static CANStatusLogger get(int busIndex){
        try{
            return get(RobotRuntimeConstants.kRobotConfiguration.getCANBusNames().get(busIndex));
        } catch (IndexOutOfBoundsException e){
            return null;
        }
    }

    /**
     * Update the logs for all CAN status loggers
     */
    public static void updateAllLogs(){
        for(CANStatusLogger logger : instances.values()){
            logger.updateLog();
        }
    }

    // ------ Actual class methods below here ------
    private List<CANDeviceID> devices = new ArrayList<>();

    // The can bus we are logging data for
    private final CANBus canBus;
    private final String logLocationPrefix;

    // List of signals that need to be checked for this CAN logger, gets populated when updatS
    private BaseStatusSignal[] ctreSignals = null;

    /**
     * Setup a new CAN status logger on some bus
     * @param busName Name of the bus we are performing logging for
     */
    private CANStatusLogger(String busName){
        this.canBus = new CANBus(busName);

        // We are logging can status to CANStatus/<bus name>/*
        logLocationPrefix = "CANStatus/" + busName + "/";
    }

    public String getBusName(){
        return this.canBus.getName();
    }

    /**
     * Register a new CAN device with our CAN logger
     * @param device The device we are registering with the logger
     */
    public void registerCANDevice(CANDeviceID device) {
        devices.add(device);
    }

    // --- Loggable Implementation ---
    @Override
    public void updateLog(String prefix) {
        logBusStatus();
        logDeviceStatuses();
    }

    private void populateCTRESignals(){

        /* ----------------- Populate the list of CTRE signals ----------------- */
        int validCTRESignalCount = 0;
        for (CANDeviceID device : devices) {
            if (device.getCTREStatusSignal() != null) {
                validCTRESignalCount++;
            }
        }
        
        if (ctreSignals == null || ctreSignals.length != validCTRESignalCount) {
            ctreSignals = new BaseStatusSignal[validCTRESignalCount];
            int index = 0;
            for (CANDeviceID device : devices) {
                if (device.getCTREStatusSignal() != null) {
                    ctreSignals[index++] = device.getCTREStatusSignal();
                }
            }
        }
    }
    
    /**
     * Get and log the state of the specified CAN bus
     */
    private void logBusStatus(){
        CANBusStatus status = canBus.getStatus();
        Logger.recordOutput(logLocationPrefix + "BusStatus", status.Status);
    }

    /**
     * Get and log the state of the specified CAN bus
     */
    private void logDeviceStatuses(){
        if(ctreSignals == null){
            populateCTRESignals();
        }
       
        // We don't really care about the result of this as we still want to update the status, THIS MUST HAPPEN UP HERE (isConnected doesn't update base status signals) 
        BaseStatusSignal.refreshAll(ctreSignals);
        
        // Log CAN device statuses
        for (CANDeviceID device : devices) {
            String deviceName = device.getDeviceName();
            String subsystemName = device.getSubsystemName();
            boolean isConnected = device.isConnected();

            Logger.recordOutput(logLocationPrefix + subsystemName + "/" + deviceName, isConnected);
            // There is a master device set so we want to log that
            if (device.getMasterDevice() != null){
                Logger.recordOutput(logLocationPrefix + subsystemName + "/" + deviceName + "/Following", device.getMasterDevice().getDeviceName());

            }
        }
       
    }

}
