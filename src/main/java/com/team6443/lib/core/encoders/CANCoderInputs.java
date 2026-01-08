// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.core.encoders;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** 
 * State of a CAN coder represented as a fixed type
 */
public class CANCoderInputs implements LoggableInputs{

    // Absolute position of the encoder in rotations
    public double absolutePositionRotations = 0.0;

    // Velocity of the encoder in rotations 
    public double velocityRotationsPerSecond = 0.0;

    // ------ LoggableInputs implementation ------
    @Override
    public void toLog(LogTable table) {
        table.put("AbsolutePositionRotations", absolutePositionRotations);
        table.put("VelocityRotationsPerSecond", velocityRotationsPerSecond);
    }

    @Override
    public void fromLog(LogTable table) {
        absolutePositionRotations = table.get("AbsolutePositionRotations", absolutePositionRotations);
        velocityRotationsPerSecond = table.get("VelocityRotationsPerSecond", velocityRotationsPerSecond);
    }

}
