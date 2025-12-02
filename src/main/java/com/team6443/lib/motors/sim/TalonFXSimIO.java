// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.motors.sim;

import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6443.lib.can.CANDeviceID;
import com.team6443.lib.config.motors.ServoMotorConfiguration;
import com.team6443.lib.motors.hardware.TalonFXIO;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/** 
 * Talon FX used to handle simulated variants of talons
 */
public class TalonFXSimIO extends TalonFXIO implements SimulatedMotorController {

    // The simulated state of the Talon FX being used 
    protected TalonFXSimState simSate;

    /**
     * Create a new TalonFX Sim IO using a ServoMotorConfiguration, this should be used pretty much everywhere except on drivetrain
     * @param device CAN Device representing the Talon Fx
     * @param motor The motor itself, the TalonFX is just a commonality wrapper
     */
    public TalonFXSimIO(CANDeviceID device, TalonFX motor){
        super(device, motor);
        simSate = talon.getSimState();
    }

    /**
     * Create a new TalonFX Sim IO using a ServoMotorConfiguration, this should be used pretty much everywhere except on drivetrain
     * @param device CAN Device representing the Talon Fx
     * @param servoMotorConfig The servo motor configuration to create the new TalonFXIO with
     */
    public TalonFXSimIO(CANDeviceID device, ServoMotorConfiguration<TalonFXConfiguration> servoMotorConfig){
        super(device, servoMotorConfig);

        talon.getSimState().Orientation =
        (servoMotorConfig.kMotorConfig.MotorOutput.Inverted == InvertedValue.Clockwise_Positive)
                ? ChassisReference.Clockwise_Positive
                : ChassisReference.CounterClockwise_Positive;
       
        simSate = talon.getSimState();
    }
    
    /**
     * Create a new TalonFX Sim IO using a TalonFXConfiguration, this should be used pretty much only be used for drive train
     * @param device CAN Device representing the Talon Fx
     * @param motorConfig The basic motor configuration for this TalonFX
     */
    public TalonFXSimIO(CANDeviceID device, TalonFXConfiguration motorConfig){
        super(device, motorConfig);

        talon.getSimState().Orientation =
        (motorConfig.MotorOutput.Inverted == InvertedValue.Clockwise_Positive)
                ? ChassisReference.Clockwise_Positive
                : ChassisReference.CounterClockwise_Positive;
       
        simSate = talon.getSimState();
    }

    /**
     * Retrieve the sim state of the Talon FX
     * @return Sim state of the Talon FX
     */
    public TalonFXSimState getSimState(){
        return simSate;
    }

    /**
     * Forcibly update the control signals for the simulated motor
     */
    @Override
    public Voltage updateControlSignal(
            Angle mechanismAngle,
            AngularVelocity mechanismVelocity,
            Angle encoderAngle,
            AngularVelocity encoderVelocity) {
        simSate.setRawRotorPosition(encoderAngle);
        simSate.setRotorVelocity(encoderVelocity);
        simSate.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

        return simSate.getMotorVoltageMeasure();
    }
}
