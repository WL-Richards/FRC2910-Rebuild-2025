// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.team6443.frc2025.RobotState;
import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.config.camera.CameraConfiguration.Location;
import com.team6443.lib.subsystems.vision.VisionInputs;
import com.team6443.lib.subsystems.vision.VisionInputs.AprilTagObservations;
import com.team6443.lib.subsystems.vision.limelight.LimelightIO;


import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  // Limelight object with its corresponding inputs
  private final List<Pair<LimelightIO, VisionInputs>> limelightsWithInputsList = new ArrayList<>();

  // What to prepend to logs from this subsystem
  private final String logPrefix;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(LimelightIO... limelights) {
    this.logPrefix = "Subsystems/VisionSubsystem";

    for (LimelightIO limelight : limelights){
      limelightsWithInputsList.add(Pair.of(limelight, new VisionInputs()));
      limelight.setLoggingPrefix(logPrefix);
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
      Logger.processInputs("RealOutputs/" + logPrefix + "/" + limelight.getConfiguration().toString(), inputs);

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
          Logger.recordOutput(logPrefix + "/VisionEstimatedRobotPose", RobotState.get().getAprilTagObservations().size() > 0 ? RobotState.get().getAprilTagObservations().get(0).robotPoseFromCamera : null);

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
}
