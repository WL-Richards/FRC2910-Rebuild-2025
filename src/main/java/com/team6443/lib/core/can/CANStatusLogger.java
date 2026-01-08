// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.core.can;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.team6443.lib.config.swerve.SwerveModuleConfiguration;
import com.team6443.lib.core.logging.Loggable;

import edu.wpi.first.wpilibj.DriverStation;

/** 
 * Class intended to managing logging of both the CAN bus itself and devices connected to the BUS
*/
public class CANStatusLogger implements Loggable {
    // ------ Singleton setup ------ 
    private static Map<String, CANStatusLogger> instances = new HashMap<String, CANStatusLogger>();

    // --- Logger update rates ---
    private static final double StatusUpdateRateDisabledHz = 100;
    private static final double StatusUpdateRateEnabledHz = 1;        

    private static boolean isRobotEnabled = false;
    private static int updateAttempts = 0; // when robot is enabled we want to try a few times before updating the logs

    /**
     * We want to know if we should be updating logs this loop, this will slow log updates when enabled
     * @return If we can update the logs this loop
     */
    private static boolean canUpdateLogs(){
        if (isRobotEnabled && updateAttempts < 50){
            updateAttempts++;
            return false;
        }

        updateAttempts = 0;
        return true;
    }

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
     * Update the logs for all CAN status loggers
     */
    public static void updateAllLogs(){
        if(canUpdateLogs()){
            if(DriverStation.isDisabled()){ // Only update CAN statuses when the robot is disabled
                for(CANStatusLogger logger : instances.values()){
                    logger.updateLog("", "");
                }
            }
        }
    }

    private static void updateLoggerFrequency(double frequencyHz){
        for(CANStatusLogger logger : instances.values()){
            for (CANDeviceID device : logger.devices){
                if(device.getCTREStatusSignal() != null){
                    device.getCTREStatusSignal().setUpdateFrequency(frequencyHz);
                }
            }
        }
    }
    
    /**
     * The robot is now in an enabled state logging update rates should be drastically reduced
     */
    public static void SetRobotEnabled(){
        isRobotEnabled = true;
        updateLoggerFrequency(StatusUpdateRateEnabledHz);
        System.out.println("[CANStatus] Robot now enabled! Update rate has been slowed.");
    }

    /**
     * The robot is now in an disabled state logging update rates should be returned to 
     */
    public static  void SetRobotDisabled(){
        isRobotEnabled = false;
        updateLoggerFrequency(StatusUpdateRateDisabledHz);
        System.out.println("[CANStatus] Robot now disabled! Update rate has increased.");

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

     /**
     * Register a CTRE swerve drivetrain with the logger
     * @param device The device we are registering with the logger
     */
    public void registerSwerveDrivetrain(
        SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain,
        List<SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>> moduleConfigurations,
        CANDeviceID gyroDevice
    ) {
        
        for (int i = 0; i < moduleConfigurations.size(); i++){
            // Set update rate of our CANDeviceID status signal to update at 100 hz
            SwerveModule<TalonFX, TalonFX, CANcoder> module = drivetrain.getModule(i);
            SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> moduleConfig = moduleConfigurations.get(i);

            // Set status signal updater for can
            moduleConfig.getkDriveMotorID().setStatusSignal(module.getDriveMotor().getSupplyVoltage(), 100);
            moduleConfig.getkSteerMotorID().setStatusSignal(module.getSteerMotor().getSupplyVoltage(), 100);
            moduleConfig.getkSteerEncoderID().setStatusSignal(module.getEncoder().getSupplyVoltage(), 100);
            gyroDevice.setStatusSignal(drivetrain.getPigeon2().getSupplyVoltage(), 100);

            // Add the three devices for it
            devices.add(moduleConfig.getkDriveMotorID());
            devices.add(moduleConfig.getkSteerMotorID());
            devices.add(moduleConfig.getkSteerEncoderID());
            devices.add(gyroDevice);
        }
        
    }

    // --- Loggable Implementation ---
    @Override
    public void updateLog(String standardPrefix, String inputPrefix) {
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
        Logger.recordOutput(logLocationPrefix + "BusUtilization", status.BusUtilization);
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

            Logger.recordOutput(logLocationPrefix + subsystemName + "/" + device.getDeviceType().toString() + "/" + deviceName, isConnected);
            // There is a master device set so we want to log that
            if (device.getMasterDevice() != null){
                Logger.recordOutput(logLocationPrefix + subsystemName + "/" + device.getDeviceType().toString() + "/" + deviceName + "/Following", device.getMasterDevice().getDeviceName());

            }
        }
       
    }

}
