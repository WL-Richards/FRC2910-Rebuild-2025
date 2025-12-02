// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.motors.sim;

import org.ironmaple.simulation.motorsims.SimulatedBattery;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team6443.lib.can.CANDeviceID;
import com.team6443.lib.config.motors.ServoMotorConfiguration;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class TalonFXWithCANcoderSimIO extends TalonFXSimIO {
    private final CANcoderSimState remoteCancoderSimState;
    private final CANDeviceID encoderCANDevice;

    /**
     * Create a new simulated Talon FX paired with a CANcoder
     * @param motorDevice CAN Device representing the motor
     * @param servoMotorConfig  ServoMotorConfiguration defining the high + low level properties of the motor
     * @param encoderDevice CAN Device representing the encoder
     * @param cancoder CANcoder object itself that we are able to rip the sim state from
     */
    public TalonFXWithCANcoderSimIO(
        CANDeviceID motorDevice, 
        ServoMotorConfiguration<TalonFXConfiguration> servoMotorConfig, 
        CANDeviceID encoderDevice,
        CANcoder cancoder){
        super(motorDevice, servoMotorConfig);
    
        this.remoteCancoderSimState = cancoder.getSimState();
        this.encoderCANDevice = encoderDevice;
    }

    /**
     * Create a new simulated Talon FX paired with a CANcoder
     * @param motorDevice CAN Device representing the motor
     * @param motorConfig  TalonFXConfiguration defining just the functionality of the motor
     * @param encoderDevice CAN Device representing the encoder
     * @param cancoder CANcoder object itself that we are able to rip the sim state from
     */
    public TalonFXWithCANcoderSimIO(
        CANDeviceID motorDevice, 
        TalonFXConfiguration motorConfig, 
        CANDeviceID encoderDevice,
        CANcoder cancoder){
        super(motorDevice, motorConfig);
    
        this.remoteCancoderSimState = cancoder.getSimState();
        this.encoderCANDevice = encoderDevice;
    }

    /**
     * Create a new simulated Talon FX paired with a CANcoder
     * @param motorDevice CAN Device representing the motor
     * @param motor  TalonFX object itself to use as the motor
     * @param encoderDevice CAN Device representing the encoder
     * @param cancoder CANcoder object itself that we are able to rip the sim state from
     */
    public TalonFXWithCANcoderSimIO(
        CANDeviceID motorDevice, 
        TalonFX motor, 
        CANDeviceID encoderDevice,
        CANcoder cancoder){
        super(motorDevice, motor);
    
        this.remoteCancoderSimState = cancoder.getSimState();
        this.encoderCANDevice = encoderDevice;
    }

    /**
     * Forcibly update the simulated control signals for both the CAN coder simulation and the motor simulation
     */
    @Override
    public Voltage updateControlSignal(
            Angle mechanismAngle,
            AngularVelocity mechanismVelocity,
            Angle encoderAngle,
            AngularVelocity encoderVelocity) {
        remoteCancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
        remoteCancoderSimState.setRawPosition(mechanismAngle);
        remoteCancoderSimState.setVelocity(mechanismVelocity);

        return super.updateControlSignal(
                mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }

}
