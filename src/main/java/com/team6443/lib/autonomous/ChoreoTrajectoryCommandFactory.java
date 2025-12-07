// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.autonomous;

import choreo.auto.AutoRoutine;

/** 
 * Factory for generating trajectories, that allow for logging
 */
public class ChoreoTrajectoryCommandFactory {
    
    public static LoggableAutoTrajectory createTrajectoryForRoutine(
        ChoreoPathing pathing, 
        AutoRoutine routine, 
        String trajectoryName
    ){
        return new LoggableAutoTrajectory(routine.trajectory(trajectoryName), pathing);
    }
}
