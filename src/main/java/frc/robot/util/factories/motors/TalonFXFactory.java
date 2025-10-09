// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.factories.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import frc.robot.util.can.CANDeviceID;
import frc.robot.util.can.CANStatusLogger;
import frc.robot.util.config.talonFX.NamedTalonFXConfiguration;
import frc.robot.util.phoenix6.Phoenix6Util;

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
    public static TalonFX createDefault(CANDeviceID device){
        NamedTalonFXConfiguration config = new NamedTalonFXConfiguration("DefaultConfiguration", device);
        return createWithConfig(device, config);
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
    private static TalonFX createWithConfig(CANDeviceID device, TalonFXConfiguration talonConfig){
        TalonFX talon = create(device);
        Phoenix6Util.applyAndCheckConfiguration(talon, talonConfig);

        // Set update rate of our CANDeviceID status signal to update at 100 hz
        device.setStatusSignal(talon.getSupplyVoltage(), 100);

        // Automatically register the Talon with the CAN status logger upon creation 
        CANStatusLogger.get(device.getBus()).registerCANDevice(device);
        return talon;
    }

    private static TalonFX create(CANDeviceID device) {
        TalonFX talon = new TalonFX(device.getDeviceID(), device.getBus());
        talon.clearStickyFaults();
        return talon;
    }
}
