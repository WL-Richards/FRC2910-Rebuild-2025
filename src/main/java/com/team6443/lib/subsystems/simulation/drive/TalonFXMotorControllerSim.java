// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.simulation.drive;

import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/** 
 * Represents a generic talon FX sim
 */
class TalonFXMotorControllerSim implements SimulatedMotorController{
    public final int id;

    private final TalonFXSimState talonFXSimState;

    public TalonFXMotorControllerSim(TalonFX talonFX) {
        this.id = talonFX.getDeviceID();
        this.talonFXSimState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
            Angle mechanismAngle,
            AngularVelocity mechanismVelocity,
            Angle encoderAngle,
            AngularVelocity encoderVelocity) {
        talonFXSimState.setRawRotorPosition(encoderAngle);
        talonFXSimState.setRotorVelocity(encoderVelocity);
        talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

        return talonFXSimState.getMotorVoltageMeasure();
    }
}
