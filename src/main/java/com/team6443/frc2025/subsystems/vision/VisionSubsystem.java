// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.team6443.frc2025.subsystems.AEMSubsystem;
import com.team6443.lib.RobotState;
import com.team6443.lib.config.camera.CameraConfiguration;

import com.team6443.lib.subsystems.vision.VisionInputs;
import com.team6443.lib.subsystems.vision.VisionInputs.AprilTagObservations;
import com.team6443.lib.subsystems.vision.limelight.LimelightIO;


import edu.wpi.first.math.Pair;


public class VisionSubsystem extends AEMSubsystem {

  // Limelight object with its corresponding inputs
  private final List<Pair<LimelightIO, VisionInputs>> limelightsWithInputsList = new ArrayList<>();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(LimelightIO... limelights) {
    super("VisionSubsystem");

    for (LimelightIO limelight : limelights){
      limelightsWithInputsList.add(Pair.of(limelight, new VisionInputs()));
    }
  }

  @Override
  public void periodic() {
    List<VisionInputs.AprilTagObservations> aprilTagObservations = new ArrayList<>();

    /**
     * For every limelight update its inputs and also determine if it had a valid pose that needs to be recorded
     */
    for (Pair<LimelightIO, VisionInputs> limelightWithInput : limelightsWithInputsList){
      LimelightIO limelight = limelightWithInput.getFirst();
      VisionInputs inputs = limelightWithInput.getSecond();

      limelight.updateInputs(inputs);
      Logger.processInputs(kLogPrefixInput + "/" + limelight.getConfiguration().toString(), inputs);
      limelight.updateLog();

      // If this input has a valid robot pose we want to add it to our observation list
      if(inputs.hasTag && inputs.robotPoseBasedOffTagLocationLatencyCompensated != null){

        // Build and add the observation to the list
        aprilTagObservations.add(
          new VisionInputs.AprilTagObservations(
            limelight.getConfiguration().toString(),
            limelight.getConfiguration().CameraLocation,
            inputs.tagID,
            inputs.robotPoseBasedOffTagLocationLatencyCompensated
          )
        );
      }
    }

    // Add the vision observations to the robot state
    RobotState.get().addVisionObservation(aprilTagObservations.toArray(new VisionInputs.AprilTagObservations[0]));

    for( AprilTagObservations observation : RobotState.get().getAprilTagObservations()){
      
      if (observation.cameraLocation == CameraConfiguration.Location.FRONT_LEFT|| observation.cameraLocation == CameraConfiguration.Location.FRONT_RIGHT){
        if(observation.tagID == 18 ){
          Logger.recordOutput(kLogPrefixStandard + "/VisionEstimatedRobotPose", RobotState.get().getAprilTagObservations().size() > 0 ? RobotState.get().getAprilTagObservations().get(0).robotPoseFromCamera : null);
        }
        break;
      }
    }
  }

  /**
   * Set the throttle value of the limelights for all cameras
   * @param throttle The throttle level to set for the cameras
   */
  public void setThrottleValue(int throttle){
    for (Pair<LimelightIO, VisionInputs> camera : limelightsWithInputsList) {
      var io = camera.getFirst();
      io.setThrottle(throttle);
    }
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
  }
}
