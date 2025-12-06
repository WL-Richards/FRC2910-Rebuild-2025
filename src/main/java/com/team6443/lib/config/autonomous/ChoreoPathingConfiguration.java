// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.autonomous;

import com.team6443.lib.config.wrappers.PIDControllerConstants;

/** 
 * Configuration used to power Choreo path following
 */
public class ChoreoPathingConfiguration {
    // --- Configurations for XY Robot Translation ---
    public PIDControllerConstants kXTranslationConfiguration;
    public PIDControllerConstants kYTranslationConfiguration;

    // --- Configurations for Robot Yaw Rotation ---
    public PIDControllerConstants kYawRotationConfiguration;

    // --- Fluent Setters ---
    
    /**
     * Updates the X-axis translation configuration on this instance.
     * @param xTranslationConfiguration The new PID constants for X translation.
     * @return This ChoreoPathingConfiguration instance for chaining.
     */
    public ChoreoPathingConfiguration withXTranslationConfiguration(PIDControllerConstants xTranslationConfiguration) {
        this.kXTranslationConfiguration = xTranslationConfiguration; // Update value
        return this;
    }
    
    /**
     * Updates the Y-axis translation configuration on this instance.
     * @param yTranslationConfiguration The new PID constants for Y translation.
     * @return This ChoreoPathingConfiguration instance for chaining.
     */
    public ChoreoPathingConfiguration withYTranslationConfiguration(PIDControllerConstants yTranslationConfiguration) {
        this.kYTranslationConfiguration = yTranslationConfiguration; // Update value
        return this;
    }
    
    /**
     * Updates the yaw rotation configuration on this instance.
     * @param yawRotationConfiguration The new PID constants for yaw rotation.
     * @return This ChoreoPathingConfiguration instance for chaining.
     */
    public ChoreoPathingConfiguration withYawRotationConfiguration(PIDControllerConstants yawRotationConfiguration) {
        this.kYawRotationConfiguration = yawRotationConfiguration; // Update value
        return this;
    }
}
