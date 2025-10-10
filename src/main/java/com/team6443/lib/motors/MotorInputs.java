// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.motors;

import org.littletonrobotics.junction.Logger;

import com.team6443.lib.logging.interfaces.Loggable;

/** 
 * Defines generic inputs to be passed to a motor
 */

public class MotorInputs implements Loggable{

    public double velocityUnitsPerSecond = 0.0;
    public double unitPosition = 0.0;
    public double appliedVolts = 0.0;
    public double currentStatorAmps = 0.0;
    public double currentSupplyAmps = 0.0;
    public double rawRotorPosition = 0.0;

    // --- Loggable interface ---
    @Override
    public void updateLog(String prefix) {
       Logger.recordOutput(prefix + "Inputs/VelocityUnitsPerSecond", velocityUnitsPerSecond);
       Logger.recordOutput(prefix + "Inputs/UnitPosition", unitPosition);
       Logger.recordOutput(prefix + "Inputs/AppliedVolts", appliedVolts);
       Logger.recordOutput(prefix + "Inputs/CurrentStatorAmps", currentStatorAmps);
       Logger.recordOutput(prefix + "Inputs/CurrentSupplyAmps", currentSupplyAmps);
       Logger.recordOutput(prefix + "Inputs/RawRotorPosition", rawRotorPosition);
    }

}
