// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.vision.limelight;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.team6443.lib.constants.FieldConstants;
import com.team6443.lib.SimulatedRobotState;
import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.config.camera.SimulatedCameraConfiguration;
import com.team6443.lib.subsystems.vision.util.AprilTagCornerPosition;
import com.team6443.lib.subsystems.vision.VisionInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;


/** 
 * Simulation code for a Limelight
 */
public class LimelightIOSim implements LimelightIO {

    // --- Configuration ---
    private final SimulatedCameraConfiguration kSimulatedCameraConfiguration;
    
    // Photonvision cameras used for simulation
    private final PhotonCamera camera;
    private final PhotonCameraSim simulatedCamera;

    // Tracks the active tag corners statically
    private final List<AprilTagCornerPosition> kTagCorners = List.of(
        new AprilTagCornerPosition(), 
        new AprilTagCornerPosition(), 
        new AprilTagCornerPosition(), 
        new AprilTagCornerPosition()
    );

    // Number of corners that are actively seen by the camera
    private int tagCornerCount = 0;

    // Current state
    private boolean hasTarget = false;
    private double xOffset = 0.0;
    private int tagID = -1;

    public LimelightIOSim(
        SimulatedCameraConfiguration config
    ){
        this.kSimulatedCameraConfiguration = config;

        // Setup the photon camera and sims
        this.camera = new PhotonCamera(this.kSimulatedCameraConfiguration.toString());
        this.simulatedCamera = new PhotonCameraSim(camera, kSimulatedCameraConfiguration.kSimCameraProperties);

        // Add the camera from the configuration to the simulated vision system
        SimulatedRobotState.get().addCameraToVisionSimulation(
            simulatedCamera, 
            new Transform3d(
                kSimulatedCameraConfiguration.kCameraConfiguration.CameraLocation.CameraPose.getTranslation(),
                kSimulatedCameraConfiguration.kCameraConfiguration.CameraLocation.CameraPose.getRotation()
            )
        );
    }

    // --- CameraIO Implementations ---
    @Override
    public CameraConfiguration getConfiguration() {
       return kSimulatedCameraConfiguration.kCameraConfiguration;
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.size() > 0){
            PhotonPipelineResult result = results.get(results.size()-1);
            inputs.hasTag = result.hasTargets();
            hasTarget = inputs.hasTag;

            if (inputs.hasTag){
                PhotonTrackedTarget target = result.getBestTarget();
                Transform3d camToTarget = target.getBestCameraToTarget(); // camera → tag in 3D
        
               
                inputs.tagID = target.fiducialId;
                tagID = inputs.tagID;

                xOffset = target.getYaw();
                inputs.horizontalRotationToTag = Rotation2d.fromDegrees(xOffset);

                // Update april tag corner positions
                List<TargetCorner> cornerPositions = target.getDetectedCorners();

                // Divide by 2 to get corner count instead of the x y counts
                tagCornerCount = cornerPositions.size();

                // ensure >= 8 because array like [x0, y0, x1, y1, etc.]
                if (cornerPositions.size() >= 4) {
                    // Then only 4 here cause we move by 2 
                    for (int i = 0; i < 4; i++) {
                        kTagCorners.get(i).x = cornerPositions.get(i).x;
                        kTagCorners.get(i).y = cornerPositions.get(i).y;
                    }
                    
                }
                inputs.tagCornerPositions = getTagCornerPositions();

                // --- Pose Estimation ---
                Pose3d tagPose = FieldConstants.getTagPose3d(inputs.tagID);

                

                if(inputs.hasTag && tagCornerCount >= 4){
                    inputs.tagHeightPixels = LimelightIO.computeTagHeightInPixels(kTagCorners);
                    inputs.tagHeightRotations = LimelightIO.computeTagHeightInRotations(
                                                                inputs.tagHeightPixels,                     // Height of tag in pixels
                                                                kSimulatedCameraConfiguration.kCameraConfiguration.CameraFOV.VerticalDegrees,   // Camera vertical FOV
                                                                kSimulatedCameraConfiguration.kCameraConfiguration.CameraResolution.YPixels     // Camera Vertical Resolution
                                                            );

                    
                    inputs.tagDistanceMeters = LimelightIO.computeDistanceToTagInMetersSimple(
                        inputs.tagHeightRotations
                    ) * kSimulatedCameraConfiguration.kCameraConfiguration.CameraDistanceScalar;    
                }

                // Nullify inputs
                else{
                    inputs.tagHeightPixels = -1;
                    inputs.tagHeightRotations = Rotation2d.kZero;
                    inputs.tagDistanceMeters = -1;
                }

                // Ensure that if we have a tag it is a good solid track, and if so compute the pose
                if (inputs.hasTag && inputs.tagDistanceMeters != -1 && inputs.tagID > FieldConstants.MIN_APRIL_TAG_ID && inputs.tagID < FieldConstants.MAX_APRIL_TAG_ID){
                    Rotation2d robotRotation = SimulatedRobotState.get().getLatestFieldRobotPose().getRotation();
                    Translation2d cameraToRobotCenter = LimelightIO.computeCameraToRobotCenter(
                        kSimulatedCameraConfiguration.kCameraConfiguration, 
                        robotRotation, 
                        inputs.horizontalRotationToTag
                    );
                    Translation2d cameraToTag = LimelightIO.computeCameraToTag(
                        kSimulatedCameraConfiguration.kCameraConfiguration, 
                        robotRotation,
                        inputs.horizontalRotationToTag,
                        inputs.tagDistanceMeters
                    );

                    VisionPoseEstimation poseEstimation = LimelightIO.computeRobotPose(
                        tagPose.toPose2d(), 
                        kSimulatedCameraConfiguration.kCameraConfiguration, 
                        inputs.tagDistanceMeters, 
                        inputs.horizontalRotationToTag,
                        robotRotation,
                        cameraToRobotCenter,
                        cameraToTag
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
            else{
                inputs.hasTag = false;
                hasTarget = inputs.hasTag;
                inputs.tagID = -1;
                tagID = inputs.tagID;
                inputs.tagHeightPixels = -1;
                inputs.tagHeightRotations = Rotation2d.kZero;
                inputs.tagDistanceMeters = -1;
                inputs.robotPoseBasedOffTagLocationLatencyCompensated = null;
                inputs.robotPoseBasedOffTagLocationLatencyUncompensated = null;
            }
            
        }
        else{
            inputs.hasTag = false;
            hasTarget = inputs.hasTag;
            inputs.tagID = -1;
            tagID = inputs.tagID;
            inputs.tagHeightPixels = -1;
            inputs.tagHeightRotations = Rotation2d.kZero;
            inputs.tagDistanceMeters = -1;
            inputs.robotPoseBasedOffTagLocationLatencyCompensated = null;
            inputs.robotPoseBasedOffTagLocationLatencyUncompensated = null;
        }
        
    }

    // --- LimelightIO Implementations ---
    @Override
    public boolean hasTarget() {
       return hasTarget;
    }

    @Override
    public double getXOffset() {
        return xOffset;
    }

    @Override
    public int getTagID() {
       return tagID;
    }

    @Override
    public List<AprilTagCornerPosition> getTagCornerPositions() {
       return kTagCorners;
    }

    @Override
    public boolean setThrottle(int throttle) {
       return true;
    }

    @Override
    public void updateLog(String standardPrefix, String inputPrefix) {
        Logger.recordOutput(standardPrefix + "/" + kSimulatedCameraConfiguration.toString() + "/NumberOfTagCorners", kTagCorners.size());
    }

}
