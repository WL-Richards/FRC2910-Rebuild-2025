// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.controllers;

import com.team6443.lib.controllers.interfaces.DriveControllable;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** 
 * Implementation of the 
*/
public class XboxControllerImplementation implements DriveControllable {
    private final CommandXboxController controller;

    public XboxControllerImplementation(int port){
        controller = new CommandXboxController(port);
    }

    @Override
    public double getThrottle() {
        return (Math.pow(Math.abs(controller.getLeftY()), 1.5))
                * Math.signum(controller.getLeftY());
    }

    @Override
    public double getStrafe() {
        return (Math.pow(Math.abs(controller.getLeftX()), 1.5))
                * Math.signum(controller.getLeftX());
    }

    @Override
    public double getRotation() {
        return (Math.pow(Math.abs(controller.getRightX()), 2.0))
                * Math.signum(controller.getRightX());
    }

    @Override
    public double getRotationY() {
        return (Math.pow(Math.abs(controller.getRightY()), 2.0))
        * Math.signum(controller.getRightY());
    }

    @Override
    public Trigger resetGyro() {
        return controller.back().and(controller.start().negate());
    }
}
