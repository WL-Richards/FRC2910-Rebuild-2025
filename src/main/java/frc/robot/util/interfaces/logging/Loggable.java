// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.interfaces.logging;

/** 
 * Interface to provide common functionality for easily logging data from robot code elements (subsystems, state machines, etc.)
 */
public interface Loggable {

    /**
     * Called to log the data from this element
     */
    public abstract void updateLog();
}
