// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.drive;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** 
 * Drive train interface describing how to interface with some drivetrain
 */
public interface DrivetrainIO {
    
    /**
     * Update the state of the drive train
     * @param inputs The inputs that are to be updated by the internal state
     */
    void updateInputs(DrivetrainInputs inputs);

    /**
     * Log the state of the swerve drive modules
     * @param state Current Swerve Drive state to be logged
     */
    void logModules(SwerveDriveState state, String prefix);

    /**
     * Resets the drive train odometry to some pose
     * @param pose
     */
    void resetOdometry(Pose2d pose);

    /**
     * Command the swerve drive to do somethings
     * @param request The operation to preform on the swerve drive
     */
    void setControl(SwerveRequest request);

    /**
     * Apply the SwerveRequest in the supplier to the swerve drive train until this command is cancelled
     * @param requestSupplier The Supplier<SwerveRequest> used to control what request is being given to the drive train
     * @param subsystemsRequired The subsystems required to drive this command (will be whatever our drive subsystem is)
     * @return The command that is being executed by this request
     */
    Command continuousRequestCommand(Supplier<SwerveRequest> requestSupplier, Subsystem... subsystemsRequired);

    /**
     * Sets how much we trust the robots reported odometry, stdevs increase as you trust teh state estimated less
     * @param xStd +/- X standard deviation in meters
     * @param yStd +/- Y standard deviation in meters
     * @param rotStd +/- theta standard deviation in radians
     */
    void setOdometryStdDevs(double xStd, double yStd, double rotStd);

    /**
     * Retrieve the swerve drive kinematics for this swerve drivetrain
     * @return The SwerveDriveKinematics object that represents this swerve drive train
     */
    SwerveDriveKinematics getSwerveKinematics();

   
}
