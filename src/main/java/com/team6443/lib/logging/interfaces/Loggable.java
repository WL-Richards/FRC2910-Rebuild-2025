// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.logging.interfaces;

/** 
 * Interface to provide common functionality for easily logging data from robot code elements (subsystems, state machines, etc.)
 */
public interface Loggable {

    /**
     * Default calls the updateLog funtion with no prefix
     */
    public default void updateLog(){
        updateLog("", "");    }

    /**
     * Called to log the data from this element
     * @param prefix What comes before this file in the Log path, this should end with a / 
     */
    public abstract void updateLog(String standardPrefix, String inputPrefix);
}
