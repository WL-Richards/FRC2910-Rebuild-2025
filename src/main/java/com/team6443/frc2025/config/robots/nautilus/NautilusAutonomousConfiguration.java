// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.config.robots.nautilus;

import com.team6443.lib.config.autonomous.ChoreoPatherConfiguration;
import com.team6443.lib.config.wrappers.PIDControllerConstants;

/**
 * Configuration for the autonomous functionality of this robot
 */
public class NautilusAutonomousConfiguration {

    // --- Choreo pathing configuration ---
    public final ChoreoPatherConfiguration kChoreoPatherConfiguration = 
        new ChoreoPatherConfiguration()
            // X Translation PID
            .withXTranslationConfiguration(
                new PIDControllerConstants()
                    .withP(10.0)
                    .withI(0.0)
                    .withD(0.0)
            )

            // Y Translation PID
            .withYTranslationConfiguration(
                new PIDControllerConstants()
                    .withP(10.0)
                    .withI(0.0)
                    .withD(0.0)
            )

            // Yaw rotation PID
            .withYawRotationConfiguration(
                new PIDControllerConstants()
                    .withP(7.5)
                    .withI(0.0)
                    .withD(0.0)
                    .withContinuousInput(-Math.PI, Math.PI)
            );
    
    
}
