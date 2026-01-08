// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.elevator;

import com.team6443.lib.core.logging.Loggable;
import com.team6443.lib.core.motors.interfaces.MotorIO;

/** Elevator IO interface */
public interface ElevatorIO  extends Loggable {

    /**
     * Get driving motors associated with the elevator
     * @return The talon FX IO leading the elevator movement motion
     */
    public MotorIO getLeadMotor();

    /**
     * Get driving motors associated with the elevator, that are following some leader
     * @return The talon FX IOs that are following the leader
     */
    public MotorIO[] getFollowerMotors();
}
