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
import com.team6443.lib.subsystems.vision.util.AprilTagCornerPosition;
import com.team6443.lib.subsystems.vision.util.limelight.LimelightHelpers;
import com.team6443.lib.subsystems.vision.VisionInputs;
import com.team6443.lib.subsystems.vision.interfaces.CameraIO;

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
public class Limelight4HardwareIO implements CameraIO {

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
    }

    public Limelight4HardwareIO(
        CameraConfiguration config,
        YearFieldConstantable yearSpecificFieldConstants,
        Supplier<Pose2d> latestFieldPoseSupplier,
        Supplier<ChassisSpeeds> latestFieldChassisSpeedSupplier
    ){
        this(config, yearSpecificFieldConstants, latestFieldPoseSupplier, latestFieldChassisSpeedSupplier, null, null);
    }

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
    public void updateInputs(VisionInputs inputs) {

        // --- Tag Information ---
        inputs.hasTag = hasTarget();
        inputs.tagID = getTagID();
        inputs.horizontalRotationToTag = getXRotationOffset();

        // ------ This will only do anything if we are on real hardware ------
        // Update april tag corner positions
        double[] cornerPositions = kTagCornerPositionsEntry.getDoubleArray(new double[0]);

        // ensure >= 8 because array like [x0, y0, x1, y1, etc.]
        if (cornerPositions.length >= 8) {
             // Then only 4 here cause we move by 2 
             for (int i = 0; i < 4; i++) {
                 kTagCorners.get(i).x = cornerPositions[i * 2];
                 kTagCorners.get(i).y = cornerPositions[i * 2 + 1];
             }
        }
        // -------------------------------------------------------------------
        inputs.tagCornerPositions = getTagCornerPositions();

        // --- Pose Estimation ---
        if (inputs.hasTag){
            Pose3d tagPose = FieldConstants.getTagPose3d(inputs.tagID, kFieldConstants);
            if(inputs.tagCornerPositions.size() >= 4){
                inputs.tagHeightPixels = LimelightIO.computeTagHeightInPixels(inputs.tagCornerPositions);
                
                inputs.tagHeightRotations = LimelightIO.computeTagHeightInRotations(
                                                        inputs.tagHeightPixels,                     // Height of tag in pixels
                                                        this.kCameraConfiguration.CameraFOV.VerticalDegrees,   // Camera vertical FOV
                                                        this.kCameraConfiguration.CameraResolution.YPixels     // Camera Vertical Resolution
                                                    );
    
                
                                                    
                inputs.tagDistanceMeters = LimelightIO.computeDistanceToTagInMetersSimple(
                    inputs.tagHeightRotations,
                    (tagPose.getZ() - FieldConstants.APRIL_TAG_HEIGHT_METERS/2) - this.kCameraConfiguration.CameraLocation.CameraPose.getZ()
                ) * this.kCameraConfiguration.CameraDistanceScalar;    
            }
    
            // Nullify inputs
            else{
                inputs.tagHeightPixels = -1;
                inputs.tagHeightRotations = Rotation2d.kZero;
                inputs.tagDistanceMeters = -1;
            }
    
            // Ensure that if we have a tag it is a good solid track, and if so compute the pose
            if (inputs.tagDistanceMeters != -1 && inputs.tagID > kFieldConstants.getMinAprilTagID() && inputs.tagID < kFieldConstants.getMaxAprilTagID()){
                
                Rotation2d robotRotation = kLatestFieldRobotPoseSupplier.get().getRotation();
                Translation2d cameraToRobotCenter = LimelightIO.computeCameraToRobotCenter(
                    this.kCameraConfiguration, 
                    robotRotation, 
                    inputs.horizontalRotationToTag
                );
                Translation2d cameraToTag = LimelightIO.computeCameraToTag(
                    this.kCameraConfiguration, 
                    robotRotation,
                    inputs.horizontalRotationToTag,
                    inputs.tagDistanceMeters
                );
    
                VisionPoseEstimation poseEstimation = LimelightIO.computeRobotPose(
                    tagPose.toPose2d(), 
                    this.kCameraConfiguration, 
                    inputs.tagDistanceMeters, 
                    inputs.horizontalRotationToTag,
                    robotRotation,
                    cameraToRobotCenter,
                    cameraToTag,
                    kLatestFieldChassisSpeedSupplier.get()
                );
    
                // This limelight has a valid robot pose computation
                inputs.robotPoseBasedOffTagLocationLatencyCompensated = poseEstimation.latencyCompensatedRobotFieldPose;
                inputs.robotPoseBasedOffTagLocationLatencyUncompensated = poseEstimation.uncompensatedRobotFieldPose;
            }
            else{
                inputs.robotPoseBasedOffTagLocationLatencyCompensated = null;
                inputs.robotPoseBasedOffTagLocationLatencyUncompensated = null;
            }
        }
    }

    @Override
    public CameraConfiguration getConfiguration() {
        return this.kCameraConfiguration;
    }
    
    // --- Loggable Implementation ---
    @Override
    public void updateLog(String standardPrefix, String inputPrefix) {
        Logger.recordOutput(standardPrefix + "/" + kCameraConfiguration.toString() + "/NumberOfTagCorners", kTagCorners.size());
    }
}
