// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.vision.io.limelight;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.team6443.frc2025.state.SimulatedRobotState;
import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.config.camera.SimulatedCameraConfiguration;
import com.team6443.lib.constants.FieldConstants;
import com.team6443.lib.constants.fields.interfaces.YearFieldConstantable;
import com.team6443.lib.subsystems.vision.util.AprilTagCornerPosition;
import com.team6443.lib.subsystems.vision.VisionInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;
/** 
 * Simulation code for a Limelight 4
 */
public class Limelight4SimIO extends Limelight4HardwareIO {

    // --- Configuration ---
    private final SimulatedCameraConfiguration kSimulatedCameraConfiguration;

    // Photonvision cameras used for simulation
    private final PhotonCamera camera;
    private final PhotonCameraSim simulatedCamera;

    // Pose estimator for constrained solvepnp
    private final PhotonPoseEstimator poseEstimator;
    private final Transform3d robotToCamera;

    public Limelight4SimIO(
        SimulatedCameraConfiguration config,
        YearFieldConstantable yearSpecificFieldConstants,
        Supplier<Pose2d> latestFieldPoseSupplier,
        Supplier<ChassisSpeeds> latestRobotChassisVelocitySupplier,
        Supplier<Rotation2d> latestRobotCameraRotationSupplier,
        Supplier<Double> latestRobotCameraAngularVelocitySupplier,
        BiConsumer<PhotonCameraSim, Transform3d> registerVisionSimulationConsumer
    ){
        super(config.kCameraConfiguration, yearSpecificFieldConstants, latestFieldPoseSupplier, latestRobotChassisVelocitySupplier, latestRobotCameraRotationSupplier, latestRobotCameraAngularVelocitySupplier);
        this.kSimulatedCameraConfiguration = config;

        // Setup the photon camera and sims
        this.camera = new PhotonCamera(this.kSimulatedCameraConfiguration.toString());
        this.simulatedCamera = new PhotonCameraSim(camera, kSimulatedCameraConfiguration.kSimCameraProperties);

        // Setup robot-to-camera transform for pose estimation
        this.robotToCamera = new Transform3d(
            kSimulatedCameraConfiguration.kCameraConfiguration.CameraLocation.CameraPose.getTranslation(),
            kSimulatedCameraConfiguration.kCameraConfiguration.CameraLocation.CameraPose.getRotation()
        );

        // Setup pose estimator for constrained solvepnp
        this.poseEstimator = new PhotonPoseEstimator(
            yearSpecificFieldConstants.getFieldLayout(),
            PhotonPoseEstimator.PoseStrategy.CONSTRAINED_SOLVEPNP,
            robotToCamera
        );

        // Register this simulated camera with the overall camera simulation
        registerVisionSimulationConsumer.accept(simulatedCamera, robotToCamera);
    }

    public Limelight4SimIO(
        SimulatedCameraConfiguration config,
        YearFieldConstantable yearSpecificFieldConstants,
        Supplier<Pose2d> latestFieldPoseSupplier,
        Supplier<ChassisSpeeds> latestRobotChassisSpeedSupplier,
        BiConsumer<PhotonCameraSim, Transform3d> registerVisionSimulationConsumer
    ){
        this(
            config, 
            yearSpecificFieldConstants, 
            latestFieldPoseSupplier, 
            latestRobotChassisSpeedSupplier, 
            null, 
            null,
            registerVisionSimulationConsumer
        );    
    }

    // --- CameraIO Implementations ---
    @Override
    public CameraConfiguration getConfiguration() {
       return kSimulatedCameraConfiguration.kCameraConfiguration;
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        // Handle updating the cameras transform if the camera rotates (like on a turret)
        if (kLatestRobotCameraRotationSupplier != null){
            Rotation2d turretRotation = kLatestRobotCameraRotationSupplier.get();
            if(turretRotation != null){
                Transform3d robotToTurret = new Transform3d(
                    Translation3d.kZero,
                    new Rotation3d(0.0, 0.0, turretRotation.getRadians())
                );

                // Assume the pose set in the camera configuration is relative to the turrets rotational origin
                Transform3d turretToCamera = new Transform3d(
                    kCameraConfiguration.CameraLocation.CameraPose.getTranslation(),
                    new Rotation3d(0.0, -kCameraConfiguration.CameraLocation.CameraPose.getRotation().getY(), 0.0)
                );

                Transform3d updatedRobotToCamera = robotToTurret.plus(turretToCamera);

                SimulatedRobotState.get().getVisionSystemSim().adjustCamera(this.simulatedCamera, updatedRobotToCamera);
            }
        }

        // Add heading data for constrained solvepnp - required every frame
        Rotation2d robotHeading = kLatestFieldRobotPoseSupplier.get().getRotation();
        poseEstimator.addHeadingData(Timer.getFPGATimestamp(), robotHeading);

        writeToTable(camera.getAllUnreadResults(), networkTable);
        super.updateInputs(inputs);
    }

    private void writeToTable(List<PhotonPipelineResult> results, NetworkTable table) {
        if (results.size() > 0) {
            PhotonPipelineResult result = results.get(results.size() - 1);

            // Use constrained solvepnp pose estimation
            Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);

            if (estimatedPose.isPresent()) {
                Pose3d robotPose = estimatedPose.get().estimatedPose;
                Pose2d robotPose2d = robotPose.toPose2d();

                List<Double> pose_data = new ArrayList<>(Arrays.asList(
                        robotPose.getX(),                                     // 0: X
                        robotPose.getY(),                                     // 1: Y
                        robotPose.getZ(),                                     // 2: Z
                        Math.toDegrees(robotPose.getRotation().getX()),       // 3: roll
                        Math.toDegrees(robotPose.getRotation().getY()),       // 4: pitch
                        robotPose2d.getRotation().getDegrees(),               // 5: yaw
                        result.metadata.getLatencyMillis(),                   // 6: latency ms
                        (double) estimatedPose.get().targetsUsed.size(),      // 7: tag count
                        0.0,                                                  // 8: tag span
                        0.0,                                                  // 9: tag dist
                        result.getBestTarget() != null ? result.getBestTarget().getArea() : 0.0  // 10: tag area
                ));

                // Add RawFiducials
                for (var target : result.getTargets()) {
                    pose_data.add((double) target.getFiducialId()); // 0: id
                    pose_data.add(target.getYaw());                 // 1: txnc
                    pose_data.add(target.getPitch());               // 2: tync
                    pose_data.add(target.getArea());                // 3: ta
                    pose_data.add(0.0);                             // 4: distToCamera
                    pose_data.add(0.0);                             // 5: distToRobot
                    pose_data.add(target.getPoseAmbiguity());       // 6: ambiguity
                }

                double[] poseArray = pose_data.stream().mapToDouble(Double::doubleValue).toArray();
                table.getEntry("botpose_wpiblue").setDoubleArray(poseArray);
                table.getEntry("botpose_orb_wpiblue").setDoubleArray(poseArray);
            }

            table.getEntry("tv").setInteger(result.hasTargets() ? 1 : 0);
            table.getEntry("cl").setDouble(result.metadata.getLatencyMillis());
        }
    }


    @Override
    public void updateLog(String standardPrefix, String inputPrefix) {
        super.updateLog(standardPrefix, inputPrefix);
    }

}
