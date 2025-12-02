// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.vision.limelight;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.team6443.frc2025.constants.FieldConstants;
import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.subsystems.vision.util.AprilTagCornerPosition;
import com.team6443.lib.subsystems.vision.VisionInputs;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Code of interfacing with a limelight 3A */
public class Limelight4IOHardware implements LimelightIO {

    // --- Logging ---
    private String logPrefix;

    // --- Network Table Entries ---
    // Network tables that the limelight 4 uses to communicate
    private final NetworkTableEntry kValidTagEntry;
    private final NetworkTableEntry kXOffsetEntry;
    private final NetworkTableEntry kTagIDEntry;
    private final NetworkTableEntry kTagCornerPositionsEntry;
    private final NetworkTableEntry kThrottleSetEntry;

    // --- Configuration ---
    private final CameraConfiguration kConfiguration;

    // --- Data ---
    // Tracks the active tag corners statically
    private final List<AprilTagCornerPosition> kTagCorners = List.of(
        new AprilTagCornerPosition(), 
        new AprilTagCornerPosition(), 
        new AprilTagCornerPosition(), 
        new AprilTagCornerPosition()
    );

    // Number of corners that are actively seen by the camera
    private int tagCornerCount = 0;

    public Limelight4IOHardware(
        CameraConfiguration config
    ){
        this.kConfiguration = config;

        // --- Retrieve the network table entries ---
        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

        kValidTagEntry = ntInstance.getTable(this.kConfiguration.CameraType.Name).getEntry("tv");
        kXOffsetEntry = ntInstance.getTable(this.kConfiguration.CameraType.Name).getEntry("tx");
        kTagIDEntry = ntInstance.getTable(this.kConfiguration.CameraType.Name).getEntry("tid");
        kTagCornerPositionsEntry = ntInstance.getTable(this.kConfiguration.CameraType.Name).getEntry("tcornxy");
        kThrottleSetEntry = ntInstance.getTable(this.kConfiguration.CameraType.Name).getEntry("throttle_set");

    }

    // --- CameraIO Implementation ---
    @Override
    public void updateInputs(VisionInputs inputs) {

        // --- Tag Information ---
        inputs.hasTag = hasTarget();
        inputs.tagID = getTagID();

        inputs.horizontalRotationToTag = getXRotationOffset();

        // Update april tag corner positions
        double[] cornerPositions = kTagCornerPositionsEntry.getDoubleArray(new double[0]);

        // Divide by 2 to get corner count instead of the x y counts
        tagCornerCount = cornerPositions.length/2;

        // ensure >= 8 because array like [x0, y0, x1, y1, etc.]
        if (cornerPositions.length >= 8) {
            // Then only 4 here cause we move by 2 
            for (int i = 0; i < 4; i++) {
                kTagCorners.get(i).x = cornerPositions[i * 2];
                kTagCorners.get(i).y = cornerPositions[i * 2 + 1];
            }
            
        }
        inputs.tagCornerPositions = getTagCornerPositions();
        Logger.recordOutput(this.logPrefix + "/" + kConfiguration.toString() + "/NumberOfTagCorners", kTagCorners.size());
        
       
        // --- Pose Estimation ---
        Pose3d tagPose = FieldConstants.getTagPose3d(inputs.tagID);
        if(inputs.hasTag && tagCornerCount >= 4){
            inputs.tagHeightPixels = LimelightIO.computeTagHeightInPixels(kTagCorners);
            inputs.tagHeightRotations = LimelightIO.computeTagHeightInRotations(
                                                        inputs.tagHeightPixels,                     // Height of tag in pixels
                                                        kConfiguration.CameraFOV.VerticalDegrees,   // Camera vertical FOV
                                                        kConfiguration.CameraResolution.YPixels     // Camera Vertical Resolution
                                                    );

            
            inputs.tagDistanceMeters = LimelightIO.computeDistanceToTagInMeters(
                inputs.tagHeightRotations, 

                // Height of april tag - limelight height from floor
                (tagPose.getZ() - FieldConstants.APRIL_TAG_HEIGHT_METERS/2) - kConfiguration.CameraLocation.CameraPose.getZ()
            ) * kConfiguration.CameraDistanceScalar;                                        
        }

        // Nullify inputs
        else{
            inputs.tagHeightPixels = -1;
            inputs.tagHeightRotations = Rotation2d.kZero;
            inputs.tagDistanceMeters = -1;
        }

        // Ensure that if we have a tag it is a good solid track, and if so compute the pose
        if (inputs.hasTag && inputs.tagDistanceMeters != -1 && inputs.tagID > FieldConstants.MIN_APRIL_TAG_ID && inputs.tagID < FieldConstants.MAX_APRIL_TAG_ID){
            // VisionPoseEstimation poseEstimation = LimelightIO.calculateRobotPose(
            //     tagPose.toPose2d(), 
            //     kConfiguration, 
            //     inputs.tagDistanceMeters, 
            //     inputs.horizontalRotationToTag
            // );

            // This limelight has a valid robot pose computation
            // inputs.robotPoseBasedOffTagLocationLatencyCompensated = poseEstimation.latencyCompensatedRobotFieldPose;
            // inputs.robotPoseBasedOffTagLocationLatencyUncompensated = poseEstimation.uncompensatedRobotFieldPose;
        }
        else{
            inputs.robotPoseBasedOffTagLocationLatencyCompensated = null;
            inputs.robotPoseBasedOffTagLocationLatencyUncompensated = null;
        }
    }

    @Override
    public CameraConfiguration getConfiguration() {
        return this.kConfiguration;
    }
    

    // --- LimelightIO Implementation ---
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
    public void setLoggingPrefix(String prefix) {
        this.logPrefix = prefix;
        
    }
}
