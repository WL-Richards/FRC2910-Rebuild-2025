// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.autonomous;

import com.team6443.lib.autonomous.ChoreoPather;
import com.team6443.lib.autonomous.wrappers.LoggableAutoTrajectory;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * AutoRoutines for this game should be created here
 */
public class AutoRoutineFactory {
    private final ChoreoPather pather;

    public AutoRoutineFactory(ChoreoPather pather){
        this.pather = pather;
    }

    /* Example autonomous using AutoRoutines and supporting trajectory logging */
    public AutoRoutine testAutoRoutine(){
        AutoRoutine routine = this.pather.getAutoFactory().newRoutine("testRoutine");

        // Create a new auto trajectory that will update the active trajecotry in the ChoreoPather so we can log the current running trajectory
        LoggableAutoTrajectory testTraj = this.pather.createTrajectory(routine, "TestPath");

        // When the routine begins, reset odometry and start the first trajectory 
        routine.active().onTrue(
            Commands.sequence(
                testTraj.resetOdometry(),
                testTraj.cmd()
            )
        );

        return routine;
    }
}
