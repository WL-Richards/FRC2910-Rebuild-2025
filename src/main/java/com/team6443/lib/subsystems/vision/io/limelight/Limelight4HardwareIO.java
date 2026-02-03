// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.vision.io.limelight;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.team6443.lib.constants.FieldConstants;
import com.team6443.lib.constants.fields.interfaces.YearFieldConstantable;
import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.subsystems.vision.util.MegatagPoseEstimate;
import com.team6443.lib.subsystems.vision.util.limelight.LimelightHelpers;
import com.team6443.lib.subsystems.vision.LimelightVisionInputs;
import com.team6443.lib.subsystems.vision.interfaces.CameraIO;
import com.team6443.lib.subsystems.vision.interfaces.LimelightIO;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** 
 * Code for interfacing with a Limelight 4 
 */
public class Limelight4HardwareIO extends LimelightIO {

    // --- Configuration ---
    protected final CameraConfiguration kCameraConfiguration;
    protected final YearFieldConstantable kFieldConstants;

    // Store suppliers for robot pose and robot speed to reference them without referencing the RobotState
    protected final Supplier<Pose2d> kLatestFieldRobotPoseSupplier;
    protected final Supplier<ChassisSpeeds> kLatestRobotChassisVelocitySupplier;

    // Allow an optional supplier to be passed in to specify the rotation of this camera (eg. used if this camera is on a turret and rotates)
    protected final Supplier<Rotation2d> kLatestRobotCameraRotationSupplier;
    protected final Supplier<Double> kLatestRobotCameraAngularVelocitySupplier;

    // --- Network Table Config ---
    // Table that contains all the limelight data 
    protected final NetworkTable networkTable;

    public Limelight4HardwareIO(
        CameraConfiguration config,
        YearFieldConstantable yearSpecificFieldConstants,
        Supplier<Pose2d> latestFieldPoseSupplier,
        Supplier<ChassisSpeeds> latestRobotChassisVelocitySupplier,
        Supplier<Rotation2d> latestRobotCameraRotationSupplier,
        Supplier<Double> latestRobotCameraAngularVelocitySupplier
    ){
        this.kCameraConfiguration = config;
        this.kFieldConstants = yearSpecificFieldConstants;
        this.kLatestFieldRobotPoseSupplier = latestFieldPoseSupplier;
        this.kLatestRobotChassisVelocitySupplier = latestRobotChassisVelocitySupplier;
        this.kLatestRobotCameraRotationSupplier = latestRobotCameraRotationSupplier;
        this.kLatestRobotCameraAngularVelocitySupplier = latestRobotCameraAngularVelocitySupplier;

        if(this.kCameraConfiguration.NetworkTableName == null){
            throw new IllegalArgumentException(String.format("CameraConfiguration.NetworkTableName not set for limelight: %s!", this.kCameraConfiguration.CameraLocation.toString()));
        }

        // --- Retrieve the network table entries ---
        this.networkTable = NetworkTableInstance.getDefault().getTable(this.kCameraConfiguration.NetworkTableName);
        updateCameraPose();
    }

    public Limelight4HardwareIO(
        CameraConfiguration config,
        YearFieldConstantable yearSpecificFieldConstants,
        Supplier<Pose2d> latestFieldPoseSupplier,
        Supplier<ChassisSpeeds> latestFieldChassisSpeedSupplier
    ){
        this(config, yearSpecificFieldConstants, latestFieldPoseSupplier, latestFieldChassisSpeedSupplier, null, null);    }

    /**
     * Update the pose of the camera based on the current state of the robot
     */
    private void updateCameraPose(){
        // Update the camera pose with the configuration set for this camera
        LimelightHelpers.setCameraPose_RobotSpace(
            kCameraConfiguration.NetworkTableName, 
            kCameraConfiguration.CameraLocation.CameraPose.getX(), 
            kCameraConfiguration.CameraLocation.CameraPose.getY(), 
            kCameraConfiguration.CameraLocation.CameraPose.getZ(), 
            Units.radiansToDegrees(kCameraConfiguration.CameraLocation.CameraPose.getRotation().getX()), 
            Units.radiansToDegrees(kCameraConfiguration.CameraLocation.CameraPose.getRotation().getY()), 
            Units.radiansToDegrees(kCameraConfiguration.CameraLocation.CameraPose.getRotation().getZ())
        );

        // Standard camera just needs this information below to handle tracking the robot orientation
        Rotation2d cameraHeading = kLatestFieldRobotPoseSupplier.get().getRotation();
        double cameraYawVelocity = Units.radiansToDegrees(kLatestRobotChassisVelocitySupplier.get().omegaRadiansPerSecond);

        // If there is a robot camera rotation supplier specified for this camera then we want to factor that information into it as well
        if (kLatestRobotCameraRotationSupplier != null && kLatestRobotCameraAngularVelocitySupplier != null){

            // Rotate the camera heading by the additional supplied rotation
            cameraHeading = cameraHeading.rotateBy(kLatestRobotCameraRotationSupplier.get());

            // Add the yaw velocity of the camera to the yaw velocity of the robot
            cameraYawVelocity = cameraYawVelocity + Units.radiansToDegrees(kLatestRobotCameraAngularVelocitySupplier.get());
        }

        // Populate the robot orientation with the correct values
        LimelightHelpers.SetRobotOrientation(
            kCameraConfiguration.NetworkTableName, 
            cameraHeading.getDegrees(),
            cameraYawVelocity, 
            0, 
            0, 
            0, 
            0
        );
    }

    // --- CameraIO Implementation ---
    @Override
    public void updateInputs(LimelightVisionInputs inputs) {
        // Update the limelight MT track information
        inputs.hasTag = LimelightHelpers.getTV(kCameraConfiguration.NetworkTableName);
        if(inputs.hasTag){
            // Update MT1 estimates
            LimelightHelpers.PoseEstimate megatag1LimelightEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(kCameraConfiguration.NetworkTableName);
            if(megatag1LimelightEstimate != null){
                inputs.megatag1PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag1LimelightEstimate);
                inputs.megatag1TagCount = megatag1LimelightEstimate.tagCount;
            }
           

            // Update MT2 estimates
            LimelightHelpers.PoseEstimate megatag2LimelightEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kCameraConfiguration.NetworkTableName);
            if(megatag2LimelightEstimate != null){
                inputs.megatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2LimelightEstimate);
                inputs.megatag2TagCount = megatag2LimelightEstimate.tagCount;
            }
           

        }
        
        // Update this cameras current pose
        updateCameraPose();
    }

    @Override
    public CameraConfiguration getConfiguration() {
        return this.kCameraConfiguration;
    }

    @Override
    public void updateLog(String standardPrefix, String inputPrefix) {
        
    }

}
