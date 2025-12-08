// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.autonomous.wrappers;

import com.team6443.lib.autonomous.ChoreoPather;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** 
 * Wrapper around the generic Choreo auto factory commands
 */
public class LoggableChoreoCommand {
    private final AutoFactory autoFactory;
    private final ChoreoPather choreoPather;
    private final String trajectoryName;

    public LoggableChoreoCommand(AutoFactory factory, String trajectoryName, ChoreoPather pathing){
        this.autoFactory = factory;
        this.choreoPather = pathing;
        this.trajectoryName = trajectoryName;
    }

    /**
     * Reset the odometery for this trajectory
     * @return
     */
    public Command resetOdometry(){
        return autoFactory.resetOdometry(trajectoryName);
    }

    /**
     * Wrapper for cmd that adds functionality for updating the currently active trajectory
     * @return The command wrapper for running the trajectory command
     */
    public Command cmd(){
        return Commands.parallel(
            Commands.runOnce(
                () -> this.choreoPather.setActiveTrajectory(
                        autoFactory.cache().loadTrajectory(trajectoryName).orElse(null)
                )
            ),                                                                      // Update the currently active trajectory
            autoFactory.trajectoryCmd(trajectoryName)                               // Run the actual trajectory command
        );
    }   
}
