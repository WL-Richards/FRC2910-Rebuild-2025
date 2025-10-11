package com.team6443.lib.phoenix6;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6443.lib.config.motors.TalonFXServoMotorConfiguration;
import com.team6443.lib.motors.TalonFXIO;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Class for handling CTRE device interactions this includes:
 * Reading errors, applying configurations
 * 
 * All methods should retry until max retry is met or the instruction suceeds
 * 
 * Amalgamation of 
 * Team 254: CTREUtil.java (https://github.com/Team254/FRC-2025-Public/blob/main/src/main/java/com/team254/lib/util/CTREUtil.java)
 * Team 2910: Phoenix6Util.java (https://github.com/FRCTeam2910/2025CompetitionRobot-Public/blob/main/src/main/java/org/frc2910/robot/util/phoenix6/Phoenix6Util.java)
 */
public final class CTREUtil {

    public static final int DEFUALT_MAX_RETRIES = 10;
    /**
     * Call a function repeatedly until the function returns true or the maxRetryCount is reached 
     * @param tryee The function that is being called to generate a status code that we are basing our state off
     * @param device The CAN device we are attempting to test on
     * @param maxRetryCount The number of attempts to make on the tryee before failing
     * @return The status code we got last (on success it will be OK)
     */
    public static StatusCode tryUntilOk(Supplier<StatusCode> tryee, int deviceID, int maxRetryCount){
        StatusCode statusCode = StatusCode.OK;
        for (int i = 0; i < maxRetryCount; ++i) {
            statusCode = tryee.get();
            if (statusCode == StatusCode.OK) break;
        }

        if (statusCode != StatusCode.OK) {
            DriverStation.reportError(
                    "Error calling "
                            + tryee
                            + " on ctre device id "
                            + deviceID
                            + ": "
                            + statusCode,
                    true);
        }

        return statusCode;
    }

    /**
     * Set the update frequncy for an array of signals, retrying until successful or the max retries is exceeded
     * @param frequncyHz Frequncy in Hz at which to update the signals
     * @param signals Array of BaseStatusSignal's that we are wanting to set the update rate for
     * @param deviceID CAN device ID of the device that these signals belong to
     * @return The status code we recieved last (on success it will be StatusCode.OK)
     */
    public static StatusCode setUpdateFrequencyForAll(double frequncyHz, BaseStatusSignal[] signals, int deviceID){
        return tryUntilOk(
            () -> BaseStatusSignal.setUpdateFrequencyForAll(frequncyHz, signals), 
            deviceID, 
            DEFUALT_MAX_RETRIES
        );
    }

    /**
     * Specify section for configuration
     */
    public final class Configuration {

        /**
         * Configuration options for motors
         */
        public final class Motors{
            /**
             * Enum to represent possible CTRE fault states with string names for easy printing
             */
            public enum FaultState {
                Hardware("Hardware"),
                OverSupplyV("OverSupplyVoltage"),
                Undervoltage("Undervoltage"),
                UnstableSupplyV("UnstableSupplyVoltage"),
                StatorCurrLimit("StatorCurrentLimit"),
                SupplyCurrLimit("SupplyCurrentLimit"),
                UnlicensedFeatureInUse("UnlicensedFeatureInUse"),
                BridgeBrownout("BridgeBrownout"),
                RemoteSensorReset("RemoteSensorReset"),
                RemoteSensorPosOverflow("RemoteSensorPosOverflow"),
                RemoteSensorDataInvalid("RemoteSensorDataInvalid"),
                FusedSensorOutOfSync("FusedSensorOutOfSync"),
                UsingFusedCANcoderWhileUnlicensed("UsingFusedCANcoderWhileUnlicensed"),
                MissingDifferentialFX("MissingDifferentialFX"),
                ReverseHardLimit("ReverseHardLimit"),
                ForwardHardLimit("ForwardHardLimit"),
                ReverseSoftLimit("ReverseSoftLimit"),
                ForwardSoftLimit("ForwardSoftLimit"),
                ProcTemp("ProcTemp"),
                DeviceTemp("DeviceTemp")
                ;
                private final String name;
                private FaultState(String name){
                    this.name = name;
                }

                @Override
                public String toString() {
                    return this.name;
                }
            }

            // ------ TalonFXIO ------

            // --- Configuration ---
            public static StatusCode applyConfiguration(TalonFXIO motor, TalonFXServoMotorConfiguration config) {
                return applyConfiguration(motor.getTalon(), config);
            }

            public static StatusCode applyConfiguration(TalonFXIO motor, TalonFXConfiguration config) {
                return applyConfiguration(motor.getTalon(), config);
            }

            public static StatusCode applyConfigurationNonBlocking(TalonFXIO motor, VoltageConfigs config) {
                return applyConfigurationNonBlocking(motor.getTalon(), config);
            }

            public static StatusCode applyConfiguration(TalonFXIO motor, HardwareLimitSwitchConfigs config) {
                return applyConfiguration(motor.getTalon(), config);

            }

            public static StatusCode applyConfiguration(TalonFXIO motor, MotionMagicConfigs config) {
                return applyConfiguration(motor.getTalon(), config);

            }

            public static StatusCode applyConfiguration(TalonFXIO motor, CurrentLimitsConfigs config) {
                return applyConfiguration(motor.getTalon(), config);
            }

            public static StatusCode getConfiguration(TalonFXIO motor, TalonFXConfiguration config) {
                return getConfiguration(motor.getTalon(), config);
            }

            // --- Error Checking ---
            public static void checkAndLogFaults(TalonFXIO motor){
                checkAndLogFaults(motor.getTalon(), motor.getCANDeviceName());
            }

            public static List<FaultState> checkFaults(TalonFXIO motor){
                return checkFaults(motor.getTalon());
            }

            // ------- TalonFX-------

            // --- Configuration ---
            public static StatusCode applyConfiguration(TalonFX motor, TalonFXConfiguration config) {
                return tryUntilOk(
                    () -> motor.getConfigurator().apply(config), 
                    motor.getDeviceID(), 
                    DEFUALT_MAX_RETRIES
                );
            }

            public static StatusCode applyConfigurationNonBlocking(TalonFX motor, VoltageConfigs config) {
                return motor.getConfigurator().apply(config, 0.01);
            }

            public static StatusCode applyConfiguration(TalonFX motor, HardwareLimitSwitchConfigs config) {
                return tryUntilOk(
                    () -> motor.getConfigurator().apply(config), 
                    motor.getDeviceID(), 
                    DEFUALT_MAX_RETRIES
                );
            }

            public static StatusCode applyConfiguration(TalonFX motor, MotionMagicConfigs config) {
                return tryUntilOk(
                    () -> motor.getConfigurator().apply(config), 
                    motor.getDeviceID(), 
                    DEFUALT_MAX_RETRIES
                );
            }
            
            public static StatusCode applyConfiguration(TalonFX motor, CurrentLimitsConfigs config) {
                return tryUntilOk(
                    () -> motor.getConfigurator().apply(config), 
                    motor.getDeviceID(), 
                    DEFUALT_MAX_RETRIES
                );
            }

            public static StatusCode getConfiguration(TalonFX motor, TalonFXConfiguration config) {
                return tryUntilOk(
                    () -> motor.getConfigurator().refresh(config),
                    motor.getDeviceID(), 
                    DEFUALT_MAX_RETRIES
                );
            }

            public static StatusCode applyConfiguration(TalonFX motor, TalonFXServoMotorConfiguration config) {
                return applyConfiguration(motor, config.config);
            }
 
            /**
             * Optimize the bus utilization for this motor by disabling all signals that have not been marked as needed 
             * @param motor The TalonFX we are optimizing
             * @return The result of the optimization of the bus
             */
            public static StatusCode optimizeBusUtilization(TalonFX motor){
                return tryUntilOk(
                    () -> motor.optimizeBusUtilization(),
                    motor.getDeviceID(), 
                    DEFUALT_MAX_RETRIES
                );
            }
            // --- Error Checking ---
            /**
             * If faults are present on the motor append them to the list that is returned
             * @param motor The motor we are checking for faults
             * @return List of fault states on the motor
             */
            public static List<FaultState> checkFaults(TalonFX motor){
                List<FaultState> states = new ArrayList<>();

                if(motor.getFault_Hardware().getValue()){
                    states.add(FaultState.Hardware);
                }
                if(motor.getFault_OverSupplyV().getValue()){
                    states.add(FaultState.OverSupplyV);
                }
                if(motor.getFault_Undervoltage().getValue()){
                    states.add(FaultState.Undervoltage);
                }
                if(motor.getFault_UnstableSupplyV().getValue()){
                    states.add(FaultState.UnstableSupplyV);
                }
                if(motor.getFault_StatorCurrLimit().getValue()){
                    states.add(FaultState.StatorCurrLimit);
                }
                if(motor.getFault_SupplyCurrLimit().getValue()){
                    states.add(FaultState.SupplyCurrLimit);
                }
                if(motor.getFault_UnlicensedFeatureInUse().getValue()){
                    states.add(FaultState.UnlicensedFeatureInUse);
                }
                if(motor.getFault_BridgeBrownout().getValue()){
                    states.add(FaultState.BridgeBrownout);
                }
                if(motor.getFault_RemoteSensorReset().getValue()){
                    states.add(FaultState.RemoteSensorReset);
                }
                if(motor.getFault_RemoteSensorPosOverflow().getValue()){
                    states.add(FaultState.RemoteSensorPosOverflow);
                }
                if(motor.getFault_RemoteSensorDataInvalid().getValue()){
                    states.add(FaultState.RemoteSensorDataInvalid);
                }
                if(motor.getFault_FusedSensorOutOfSync().getValue()){
                    states.add(FaultState.FusedSensorOutOfSync);
                }
                if(motor.getFault_UsingFusedCANcoderWhileUnlicensed().getValue()){
                    states.add(FaultState.UsingFusedCANcoderWhileUnlicensed);
                }
                if(motor.getFault_MissingDifferentialFX().getValue()){
                    states.add(FaultState.MissingDifferentialFX);
                }
                if(motor.getFault_ReverseHardLimit().getValue()){
                    states.add(FaultState.ReverseHardLimit);
                }
                if(motor.getFault_ForwardHardLimit().getValue()){
                    states.add(FaultState.ForwardHardLimit);
                }
                if(motor.getFault_ReverseSoftLimit().getValue()){
                    states.add(FaultState.ReverseSoftLimit);
                }
                if(motor.getFault_ForwardSoftLimit().getValue()){
                    states.add(FaultState.ForwardSoftLimit);
                }
                if(motor.getFault_ProcTemp().getValue()){
                    states.add(FaultState.ProcTemp);
                }
                if(motor.getFault_DeviceTemp().getValue()){
                    states.add(FaultState.DeviceTemp);
                }

                return states;
            }
        
            /**
             * Retrieves a list of faults from a given motor and if there are any it will print an error to the driver station
             * @param motor The motor that the faults are being checked for on
             * @param motorName The name of the motor to allow for better printouts
             */
            public static void checkAndLogFaults(TalonFX motor, String motorName){
                StringBuilder sb = new StringBuilder();
                for(FaultState state : checkFaults(motor)){
                    sb.append(state.toString()).append(", ");
                }

                if (!sb.isEmpty()) {
                    DriverStation.reportError("Talon faults occurred on the following Talon: " + motorName + "! Faults: " + sb, false);
                }
            }
        }
        
        /**
         * Configuration options for encoders
         */
        public final class Encoders {

            /**
             * Fault states for CTRE Encoders
             */
            public enum FaultState {
                Hardware("Hardware"),
                Undervoltage("Undervoltage"),
                BootDuringEnable("BootDuringEnable"),
                UnlicensedFeatureInUse("UnlicensedFeatureInUse"),
                BadMagnet("BadMagnet"),
                ;
                private final String name;
                private FaultState(String name){
                    this.name = name;
                }

                @Override
                public String toString() {
                    return this.name;
                }
            }

            // ------- CANCoder -------

            // --- Configuration ---
            public static StatusCode applyConfiguration(CANcoder cancoder, CANcoderConfiguration config) {
                return tryUntilOk(
                    () -> cancoder.getConfigurator().apply(config), 
                    cancoder.getDeviceID(), 
                    DEFUALT_MAX_RETRIES
                );
            }

            // --- Error Checking ---
            /**
             * If faults are present on the encoder append them to the list that is returned
             * @param encoder The encoder we are checking for faults
             * @return List of fault states on the encoder
             */
            public static List<FaultState> checkFaults(CANcoder encoder){
                List<FaultState> states = new ArrayList<>();

    
                if(encoder.getFault_Hardware().getValue()){
                    states.add(FaultState.Hardware);
                }
                if(encoder.getFault_Undervoltage().getValue()){
                    states.add(FaultState.Undervoltage);
                }
                if(encoder.getFault_BootDuringEnable().getValue()){
                    states.add(FaultState.BootDuringEnable);
                }
                if(encoder.getFault_UnlicensedFeatureInUse().getValue()){
                    states.add(FaultState.UnlicensedFeatureInUse);
                }
                if(encoder.getFault_BadMagnet().getValue()){
                    states.add(FaultState.BadMagnet);
                }

                return states;
            }
        
            public static void checkAndLogFaults(CANcoder encoder, String encoderName){
                StringBuilder sb = new StringBuilder();
                for(FaultState state : checkFaults(encoder)){
                    sb.append(state.toString()).append(", ");
                }
                
                if (!sb.isEmpty()) {
                    DriverStation.reportError("CANcoder faults occurred on the following CANCoder: " + encoderName + "! Faults: " + sb, false);
                }
            }
        }
    }
}
