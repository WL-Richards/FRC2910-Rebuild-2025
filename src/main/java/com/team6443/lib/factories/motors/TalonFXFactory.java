// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.factories.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.team6443.lib.can.CANDeviceID;
import com.team6443.lib.can.CANStatusLogger;
import com.team6443.lib.motors.TalonFXIO;
import com.team6443.lib.phoenix6.CTREUtil;

/** 
 * Based on some input descriptors create
 */
public class TalonFXFactory {
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
    public static final InvertedValue INVERT_VALUE = InvertedValue.CounterClockwise_Positive;
    public static final double NEUTRAL_DEADBAND = 0.04;
 
    /**
     * Create a new TalonFX with the default configuration described below and link it with the given CANDeviceID
     * @param device CANDeviceID that the TalonFX is object is being created from
     * @return The newly created Talon FX
     */
    public static TalonFXIO createDefault(CANDeviceID device){
        return createWithConfig(device, getDefaultConfig());
    }

    /**
     * Retrieve the default TalonFX configuration we should use when we call createDefault
     * @return Default TalonFX config
     */
    public static TalonFXConfiguration getDefaultConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NEUTRAL_MODE;
        config.MotorOutput.Inverted = INVERT_VALUE;
        config.MotorOutput.DutyCycleNeutralDeadband = NEUTRAL_DEADBAND;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = -1.0;

        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimitEnable = false;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.FeedbackRotorOffset = 0;
        config.Feedback.SensorToMechanismRatio = 1;

        config.HardwareLimitSwitch.ForwardLimitEnable = false;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
        config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitEnable = false;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

        config.Audio.BeepOnBoot = true;

        return config;
    }

    /**
     * Create a new TalonFX with the talon FX configuration supplied
     * @param device CANDevice the represents the Talon being created
     * @param talonConfig The config that should be applied to the talon after its created
     * @return
     */
    private static TalonFXIO createWithConfig(CANDeviceID device, TalonFXConfiguration talonConfig){
        TalonFXIO talon = create(device);
        CTREUtil.Configuration.Motors.applyConfiguration(talon, talonConfig);

        // Set update rate of our CANDeviceID status signal to update at 100 hz
        device.setStatusSignal(talon.getTalon().getSupplyVoltage(), 100);

        // Automatically register the Talon with the CAN status logger upon creation 
        CANStatusLogger.get(device.getBus()).registerCANDevice(device);
        return talon;
    }

    /**
     * Create a new TalonFXIO
     * @param device CAN device that represents this talon
     * @return The newly created TalonFXIO
     */
    private static TalonFXIO create(CANDeviceID device) {
        TalonFXIO talon = new TalonFXIO(device);
        talon.getTalon().clearStickyFaults();
        return talon;
    }
}
