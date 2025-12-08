// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.autonomous.wrappers;

import com.team6443.lib.autonomous.ChoreoPather;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** 
 * Wrapper around the Choreo AutoTrajectory 
 */
public class LoggableAutoTrajectory {
    private final AutoTrajectory trajectory;
    private final ChoreoPather pathing;

    public LoggableAutoTrajectory(AutoTrajectory baseTraj, ChoreoPather pathing){
        this.trajectory = baseTraj;
        this.pathing = pathing;
    }

    /**
     * Reset the odometery for this trajectory
     * @return
     */
    public Command resetOdometry(){
        return trajectory.resetOdometry();
    }

    /**
     * Wrapper for cmd that adds functionality for updating the currently active trajectory
     * @return The command wrapper for running the trajectory command
     */
    public Command cmd(){
        return Commands.parallel(
            Commands.runOnce(() ->this.pathing.setActiveTrajectory(trajectory.getRawTrajectory())),     // Update the currently active trajectory
            trajectory.cmd()                                                                            // Run the actual trajectory command
        );
    }
}
