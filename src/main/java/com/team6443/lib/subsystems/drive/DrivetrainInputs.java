// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Current state of the drive train
 */
public class DrivetrainInputs extends SwerveDriveState implements LoggableInputs{
    public double gyroYawAngle = 0.0;

    public double yawAngularVelocity = 0.0;
    public double rollAngularVelocity = 0.0;
    public double pitchAngularVelocity = 0.0;

    public double pitch = 0.0;
    public double roll = 0.0;

    public double accelX = 0.0;
    public double accelY = 0.0;

    public DrivetrainInputs() {
        this.Pose = Pose2d.kZero;
        this.Speeds = new ChassisSpeeds();
        this.ModuleStates = new SwerveModuleState[0]; 
        this.ModuleTargets = new SwerveModuleState[0];
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
        table.put("Pose", Pose);
        table.put("Speeds", Speeds);
        table.put("OdometryPeriod", OdometryPeriod);
        table.put("Timestamp", Timestamp);
        
        // Log Module States (Struct array support is best, manual fallback shown)
        table.put("ModuleStates", ModuleStates); 
        table.put("ModuleTargets", ModuleTargets);
        
        table.put("YawAngularVelocity", yawAngularVelocity);
        table.put("RollAngularVelocity", rollAngularVelocity);
        table.put("PitchAngularVelocity", pitchAngularVelocity);

        table.put("Pitch", pitch);
        table.put("Roll", roll);

        table.put("AccelerationX", accelX);
        table.put("AccelerationY", accelY);
    }

    @Override
    public void fromLog(LogTable table) {
        gyroYawAngle = table.get("RobotYawAngle", gyroYawAngle);
        Pose = table.get("Pose", Pose);
        Speeds = table.get("Speeds", Speeds);
        OdometryPeriod = table.get("OdometryPeriod", OdometryPeriod);
        Timestamp = table.get("Timestamp", Timestamp);
        
        ModuleStates = table.get("ModuleStates", ModuleStates);
        ModuleTargets = table.get("ModuleTargets", ModuleTargets);

        yawAngularVelocity = table.get("YawAngularVelocity", yawAngularVelocity);
        rollAngularVelocity = table.get("RollAngularVelocity", rollAngularVelocity);
        pitchAngularVelocity = table.get("PitchAngularVelocity", pitchAngularVelocity);
        
        pitch = table.get("Pitch", pitch);
        roll = table.get("Roll", roll);
        accelX = table.get("AccelerationX", accelX);
        accelY = table.get("AccelerationY", accelY);
    }
}
