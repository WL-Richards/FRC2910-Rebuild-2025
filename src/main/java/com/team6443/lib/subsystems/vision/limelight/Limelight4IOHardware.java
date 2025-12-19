// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.vision.limelight;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.team6443.lib.constants.FieldConstants;
import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.subsystems.vision.util.AprilTagCornerPosition;
import com.team6443.lib.subsystems.vision.VisionInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Code of interfacing with a limelight 3A */
public class Limelight4IOHardware implements LimelightIO {

    // --- Network Table Entries ---
    // Network tables that the limelight 4 uses to communicate
    private final NetworkTableEntry kValidTagEntry;
    private final NetworkTableEntry kXOffsetEntry;
    private final NetworkTableEntry kTagIDEntry;
    private final NetworkTableEntry kTagCornerPositionsEntry;
    private final NetworkTableEntry kThrottleSetEntry;

    // --- Configuration ---
    private final CameraConfiguration kCameraConfiguration;

    // --- Data ---
    // Tracks the active tag corners statically
    private final List<AprilTagCornerPosition> kTagCorners = List.of(
        new AprilTagCornerPosition(), 
        new AprilTagCornerPosition(), 
        new AprilTagCornerPosition(), 
        new AprilTagCornerPosition()
    );

    // Store suppliers for robot pose and robot speed to reference them without referencing the RobotState
    private final Supplier<Pose2d> kLatestRobotPoseSupplier;
    private final Supplier<ChassisSpeeds> kLatestFieldChassisSpeedSupplier;

    public Limelight4IOHardware(
        CameraConfiguration config,
        Supplier<Pose2d> latestFieldPoseSupplier,
        Supplier<ChassisSpeeds> latestFieldChassisSpeedSupplier
    ){
        this.kCameraConfiguration = config;
        this.kLatestRobotPoseSupplier = latestFieldPoseSupplier;
        this.kLatestFieldChassisSpeedSupplier = latestFieldChassisSpeedSupplier;

        // --- Retrieve the network table entries ---
        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

        kValidTagEntry = ntInstance.getTable(this.kCameraConfiguration.CameraType.Name).getEntry("tv");
        kXOffsetEntry = ntInstance.getTable(this.kCameraConfiguration.CameraType.Name).getEntry("tx");
        kTagIDEntry = ntInstance.getTable(this.kCameraConfiguration.CameraType.Name).getEntry("tid");
        kTagCornerPositionsEntry = ntInstance.getTable(this.kCameraConfiguration.CameraType.Name).getEntry("tcornxy");
        kThrottleSetEntry = ntInstance.getTable(this.kCameraConfiguration.CameraType.Name).getEntry("throttle_set");

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
        if(inputs.hasTag && inputs.tagCornerPositions.size() >= 4){
            inputs.tagHeightPixels = LimelightIO.computeTagHeightInPixels(inputs.tagCornerPositions);
            
            inputs.tagHeightRotations = LimelightIO.computeTagHeightInRotations(
                                                    inputs.tagHeightPixels,                     // Height of tag in pixels
                                                    this.kCameraConfiguration.CameraFOV.VerticalDegrees,   // Camera vertical FOV
                                                    this.kCameraConfiguration.CameraResolution.YPixels     // Camera Vertical Resolution
                                                );

            
            inputs.tagDistanceMeters = LimelightIO.computeDistanceToTagInMetersSimple(
                inputs.tagHeightRotations
            ) * this.kCameraConfiguration.CameraDistanceScalar;    
        }

        // Nullify inputs
        else{
            inputs.tagHeightPixels = -1;
            inputs.tagHeightRotations = Rotation2d.kZero;
            inputs.tagDistanceMeters = -1;
        }

        // Ensure that if we have a tag it is a good solid track, and if so compute the pose
        if (inputs.hasTag && inputs.tagDistanceMeters != -1 && inputs.tagID > FieldConstants.k2025FieldConstants.getMinAprilTagID() && inputs.tagID < FieldConstants.k2025FieldConstants.getMaxAprilTagID()){
            Pose3d tagPose = FieldConstants.getTagPose3d(inputs.tagID, FieldConstants.k2025FieldConstants);
            Rotation2d robotRotation = kLatestRobotPoseSupplier.get().getRotation();
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

    @Override
    public CameraConfiguration getConfiguration() {
        return this.kCameraConfiguration;
    }
    

    // --- LimelightIOHardware Implementation ---
    @Override
    public boolean hasTarget() {
        return kValidTagEntry.getInteger(0) == 1;
    }

    @Override
    public double getXOffset() {
        return kXOffsetEntry.getDouble(0.0);
    }

    @Override
    public int getTagID() {
        return (int)kTagIDEntry.getInteger(-1);
    }

    @Override
    public List<AprilTagCornerPosition> getTagCornerPositions() {
        return kTagCorners;
    }

    @Override
    public boolean setThrottle(int throttle) {
        return kThrottleSetEntry.setNumber(throttle);
    }

    @Override
    public void updateLog(String standardPrefix, String inputPrefix) {
        Logger.recordOutput(standardPrefix + "/" + kCameraConfiguration.toString() + "/NumberOfTagCorners", kTagCorners.size());
    }
}
