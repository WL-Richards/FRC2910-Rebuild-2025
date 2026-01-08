// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.constants;

import java.util.Optional;

import com.team6443.frc2025.config.RobotConfig;
import com.team6443.frc2025.config.RobotID;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Common functionality that every robot every year will need to have if this pattern is followed
 */
public class RobotRuntimeConstants {

    /**
     * Tracks the current runtime state of the robot
     */
    public enum RuntimeMode {
        /* This is real on robot code running */
        REAL,

        /* This code is running in a simulation */
        SIM,

        /* This code is replaying from a log file */
        REPLAY
    }

    // Determine the runtime mode of this bot, real bot, simulated bot or replaying log file
    public static final RuntimeMode kCurrentRuntimeMode = RobotBase.isReal() ? RuntimeMode.REAL : RuntimeMode.SIM;

    // What robot is this code currently running on
    public static final RobotID kRobotIdentification = RobotID.getIdentification();

    // What configuration is actually in use
    public static final RobotConfig kRobotConfiguration = RobotConfig.getRobotConstants(kRobotIdentification);

    /**
     * Are on the red alliance currently
     * @return true if alliance assigned and red alliance
     */
    public static boolean isRedAlliance(){
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }

    /**
     * Are on the blue alliance currently
     * @return true if alliance assigned and blue alliance
     */
    public static boolean isBlueAlliance(){
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().equals(Optional.of(Alliance.Blue));
    }
}
