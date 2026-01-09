// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.core.motors.io;

import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6443.lib.config.motors.MotorConfiguration;
import com.team6443.lib.core.can.CANDeviceID;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/** 
 * Talon FX used to handle simulated variants of talons
 */
public class TalonFXSimIO extends TalonFXHardwareIO implements SimulatedMotorController {

    // The simulated state of the Talon FX being used 
    protected TalonFXSimState simSate;


    /**
     * Create a new TalonFX Sim IO using a ServoMotorConfiguration, this should be used pretty much everywhere except on drivetrain
     * @param device CAN Device representing the Talon Fx
     * @param servoMotorConfig The servo motor configuration to create the new TalonFXIO with
     */
    public TalonFXSimIO(MotorConfiguration<TalonFXConfiguration> servoMotorConfig){
        super(servoMotorConfig);

        simSate = talon.getSimState();
        simSate.Orientation = TalonFXSimIO.computeSimMotorOrientation(servoMotorConfig.kMotorConfig.MotorOutput.Inverted);
    }
    
    /**
     * Create a new TalonFX Sim IO using a TalonFXConfiguration, this should be used pretty much only be used for drive train
     * @param device CAN Device representing the Talon Fx
     * @param motorConfig The basic motor configuration for this TalonFX
     */
    public TalonFXSimIO(CANDeviceID device, TalonFXConfiguration motorConfig){
        super(device, motorConfig);

        simSate = talon.getSimState();
        simSate.Orientation = TalonFXSimIO.computeSimMotorOrientation(motorConfig.MotorOutput.Inverted);    
    }

    /**
     * Create a new TalonFX Sim IO using a raw motor, this should pretty much ONLY be used on the drivetrain
     * @param motor The motor itself, the TalonFXIO is just a commonality wrapper
     */
    public TalonFXSimIO(TalonFX motor){
        super(motor);
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
     * Forcibly updates the internal simulation state of the motor using measured
     * encoder values and battery voltage, then calculates and returns the
     * corresponding **motor voltage measure**.
     *
     * This method is intended for use within a simulation environment (Sim)
     * to synchronize the simulated motor model's state (position, velocity,
     * supply voltage) with the external measurements being fed to it (e.g.,
     * from an encoder or another physics model).
     *
     * @param mechanismAngle The current angular position of the mechanism (e.g., arm, wheel)
     * that the motor is driving. Not directly used in the current
     * implementation, but included for API completeness.
     * @param mechanismVelocity The current angular velocity of the mechanism.
     * Not directly used in the current implementation.
     * @param encoderAngle The latest angular position reading from the motor's encoder.
     * This is used to set the motor's raw rotor position in the simulation.
     * @param encoderVelocity The latest angular velocity reading from the motor's encoder.
     * This is used to set the motor's rotor velocity in the simulation.
     * @return A {@link Voltage} object representing the calculated motor voltage measure
     * that should be applied to the motor for the next simulation step, based on
     * the updated state.
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

    /**
     * Compute the proper chassis orientation given some motor inverted value
     * @param value The inverted direction of the motor
     * @return The chassis centric orientation of the motor
     */
    private static ChassisReference computeSimMotorOrientation(InvertedValue value){
        return (value == InvertedValue.Clockwise_Positive)
        ? ChassisReference.Clockwise_Positive
        : ChassisReference.CounterClockwise_Positive;
    }

}
