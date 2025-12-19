// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.vision.limelight;

import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.config.camera.SimulatedCameraConfiguration;
import com.team6443.lib.constants.FieldConstants;
import com.team6443.lib.constants.interfaces.YearFieldConstantable;
import com.team6443.lib.subsystems.vision.util.AprilTagCornerPosition;
import com.team6443.lib.subsystems.vision.VisionInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


/** 
 * Simulation code for a Limelight
 */
public class LimelightIOSim extends Limelight4IOHardware {

    // --- Configuration ---
    private final SimulatedCameraConfiguration kSimulatedCameraConfiguration;
    
    // Photonvision cameras used for simulation
    private final PhotonCamera camera;
    private final PhotonCameraSim simulatedCamera;

    private double trueCameraDistance = Double.NaN;

    // Tracks the active tag corners statically
    private final List<AprilTagCornerPosition> kTagCorners = List.of(
        new AprilTagCornerPosition(), 
        new AprilTagCornerPosition(), 
        new AprilTagCornerPosition(), 
        new AprilTagCornerPosition()
    );

    // Current state
    private boolean hasTarget = false;
    private double xOffset = 0.0;
    private int tagID = -1;

    public LimelightIOSim(
        SimulatedCameraConfiguration config,
        YearFieldConstantable yearSpecificFieldConstants,
        Supplier<Pose2d> latestFieldPoseSupplier,
        Supplier<ChassisSpeeds> latestFieldChassisSpeedSupplier,
        BiConsumer<PhotonCameraSim, Transform3d> registerVisionSimulationConsumer
    ){
        super(config.kCameraConfiguration, yearSpecificFieldConstants, latestFieldPoseSupplier, latestFieldChassisSpeedSupplier);
        this.kSimulatedCameraConfiguration = config;

        // Setup the photon camera and sims
        this.camera = new PhotonCamera(this.kSimulatedCameraConfiguration.toString());
        this.simulatedCamera = new PhotonCameraSim(camera, kSimulatedCameraConfiguration.kSimCameraProperties);

        registerVisionSimulationConsumer.accept(
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
            hasTarget = result.hasTargets();
           
            if (hasTarget){
                PhotonTrackedTarget target = result.getBestTarget();
    
                tagID = target.fiducialId;
                xOffset = target.getYaw();

                // Update april tag corner positions
                List<TargetCorner> cornerPositions = target.getDetectedCorners();

                // ensure >= 8 because array like [x0, y0, x1, y1, etc.]
                if (cornerPositions.size() >= 4) {
                    // Then only 4 here cause we move by 2 
                    for (int i = 0; i < 4; i++) {
                        kTagCorners.get(i).x = cornerPositions.get(i).x;
                        kTagCorners.get(i).y = cornerPositions.get(i).y;
                    }
                }
                
                Pose2d robotPose = kLatestRobotPoseSupplier.get();
                Translation3d currentCameraFieldPosition = new Translation3d(robotPose.getTranslation().getX(), robotPose.getTranslation().getY(), 0).plus(kSimulatedCameraConfiguration.kCameraConfiguration.CameraLocation.CameraPose.getTranslation().rotateBy(new Rotation3d(robotPose.getRotation())));
                trueCameraDistance = currentCameraFieldPosition.getDistance(FieldConstants.getTagPose3d(tagID, kFieldConstants).getTranslation());
            }
            else{
                trueCameraDistance = Double.NaN;
            }
        }
        
        super.updateInputs(inputs);
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
        Logger.recordOutput(standardPrefix + "/" + kSimulatedCameraConfiguration.toString() + "/TrueDistanceToTarget", trueCameraDistance);
    }

}
