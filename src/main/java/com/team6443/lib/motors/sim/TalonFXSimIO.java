// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.motors.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6443.lib.can.CANDeviceID;
import com.team6443.lib.config.motors.ServoMotorConfiguration;
import com.team6443.lib.motors.hardware.TalonFXIO;

/** 
 * Talon FX used to handle simulated variants of talons
 */
public class TalonFXSimIO extends TalonFXIO {
    protected TalonFXSimState simSate;

    public TalonFXSimIO(CANDeviceID device, ServoMotorConfiguration<TalonFXConfiguration> servoMotorConfig){
        super(device, servoMotorConfig);
        talon.getSimState().Orientation =
                    (servoMotorConfig.motorConfig.MotorOutput.Inverted == InvertedValue.Clockwise_Positive)
                            ? ChassisReference.Clockwise_Positive
                            : ChassisReference.CounterClockwise_Positive;

        simSate = talon.getSimState();
    }

    public TalonFXSimState getSimState(){
        return simSate;
    }
}
