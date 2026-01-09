// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.motors;

import com.team6443.lib.config.encoders.CANCoderConfiguration;

/** 
 * Servo motor subsystem implementation with the addition of a CAN coder
 * 
 * C is the motor configuration intended to be used, eg. TalonFXConfiguration, etc.
 */
public class MotorCANCoderConfiguration<C> extends MotorConfiguration<C> {
    public CANCoderConfiguration canCoderConfig = new CANCoderConfiguration();

    // This is the ratio from cancoder to units.
    // cancoder rotations * by this ratio should = units of subsystem.
    public double CANCoderRotationToUnitRatio = 1.0;

    // The gear ratio that is inbetwee the CAN coder and the output
    public double CANCoderGearRatioSim = 1.0;

    // Conversion factor for converting between the desired output units and rotations of the motor
    public double CANCoderUnitToRotorRotationRatioSim = 1.0;

    // Are we using a fused CAN coder for this motor configuration
    public boolean isFusedCANCoder = false;

    /**
     * Convert the current encoder rotations to the real-world units specified scaled by the unitToRotorRotationRation defined in the config
     * @param rotorRotations Encoder rotation value we want to convert into the in-use units
     * @return The rotations converted into some units as defined in the config
     */
    public double getEncoderRotationsToUnits(double rotorRotations){
        return rotorRotations * this.kUnitToRotorRotationRatio;
    }

    /**
     * Convert the current units into encoder rotations using the defined conversion ratio
     * @param units Units we want to convert to rotor rotations
     * @return The resulting Encoder rotations 
     */
    public double getUnitsToEncoderRotations(double units){
        return units / this.kUnitToRotorRotationRatio;
    }

    public MotorCANCoderConfiguration<C> withConfig(C config){
        super.withConfig(config);
        return this;
    }
}
