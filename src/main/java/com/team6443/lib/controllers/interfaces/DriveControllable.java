// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.controllers.interfaces;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** 
 * Interface for representing the standard drive states that can be altered by the controller
 */
public interface DriveControllable {
    public double getThrottle();
    public double getStrafe();

    public double getRotation();
    public double getRotationY();

    public Trigger resetGyro();
}
