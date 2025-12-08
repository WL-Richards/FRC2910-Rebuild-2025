// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.autonomous;

import com.team6443.lib.autonomous.ChoreoPather;
import com.team6443.lib.autonomous.wrappers.LoggableChoreoCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** 
 * Auto commands should created here
 */
public class AutoCommandFactory {
    private final ChoreoPather pather;

    public AutoCommandFactory(ChoreoPather choreo){
        this.pather = choreo;
    }

    /* Example autonomous using an AutoFactory and supporting trajectory logging */
    public Command testAutoCommand(){
        // Create a loggable trajectory command for the test auto path
        LoggableChoreoCommand testTrajectory = this.pather.createTrajectory(
                                                                            this.pather.getAutoFactory(), 
                                                                            "TestPath"
                                                                            );
        return Commands.sequence(
        testTrajectory.resetOdometry(),
        testTrajectory.cmd()
        );
    }
}
