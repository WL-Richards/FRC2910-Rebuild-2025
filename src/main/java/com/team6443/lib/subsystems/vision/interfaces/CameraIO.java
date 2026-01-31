// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.vision.interfaces;

import com.team6443.lib.core.logging.Loggable;

import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.subsystems.vision.VisionInputs;

/** 
 * Basic camera object
 */
public interface CameraIO extends Loggable {

    /**
     * Get the configuration for this camera
     * @return
     */
    public CameraConfiguration getConfiguration();

    /**
     * Update the state of this camera
     * @param inputs Vision inputs to be updated
     */
    public void updateInputs(VisionInputs inputs);
}
