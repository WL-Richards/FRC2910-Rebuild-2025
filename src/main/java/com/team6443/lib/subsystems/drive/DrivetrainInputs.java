// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Current state of the drive train
 */
public class DrivetrainInputs extends SwerveDriveState implements LoggableInputs{
    public double gyroYawAngle = 0.0;

    public DrivetrainInputs() {
        this.Pose = Pose2d.kZero;
    }

    /**
     * Load the state from a SwerveDriveState
     * @param stateIn The state we are loading from
     */
    public void fromSwerveDriveState(SwerveDriveState stateIn) {
        this.Pose = stateIn.Pose;
        this.SuccessfulDaqs = stateIn.SuccessfulDaqs;
        this.FailedDaqs = stateIn.FailedDaqs;
        this.ModuleStates = stateIn.ModuleStates;
        this.ModuleTargets = stateIn.ModuleTargets;
        this.Speeds = stateIn.Speeds;
        this.OdometryPeriod = stateIn.OdometryPeriod;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("RobotYawAngle", gyroYawAngle);
    }

    @Override
    public void fromLog(LogTable table) {
        gyroYawAngle = table.get("RobotYawAngle", gyroYawAngle);
    }
}
