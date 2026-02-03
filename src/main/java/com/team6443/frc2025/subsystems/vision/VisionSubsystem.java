// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.team6443.frc2025.constants.RobotRuntimeConstants;
import com.team6443.frc2025.state.RobotState;
import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.subsystems.AEMSubsystem;
import com.team6443.lib.subsystems.vision.LimelightVisionInputs;
import com.team6443.lib.subsystems.vision.interfaces.CameraIO;
import com.team6443.lib.subsystems.vision.interfaces.LimelightIO;
import com.team6443.lib.subsystems.vision.io.limelight.Limelight4HardwareIO;
import com.team6443.lib.subsystems.vision.util.VisionFieldPoseEstimate;

import edu.wpi.first.math.Pair;

public class VisionSubsystem extends AEMSubsystem {

  // Limelight object with its corresponding inputs
  private final List<Pair<Limelight4HardwareIO, LimelightVisionInputs>> limelightsWithInputsList = new ArrayList<>();
  private final List<VisionFieldPoseEstimate> limelightFieldPoseEstimates;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(Limelight4HardwareIO... limelights) {
    super("VisionSubsystem");

    for (Limelight4HardwareIO limelight : limelights){
      limelightsWithInputsList.add(Pair.of(limelight, new LimelightVisionInputs()));
    }

    limelightFieldPoseEstimates = new ArrayList<>(limelightsWithInputsList.size());
  }

  @Override
  public void periodic() {
    limelightFieldPoseEstimates.clear();

    /**
     * For every limelight update its inputs and also determine if it had a valid pose that needs to be recorded
     */
    for (Pair<Limelight4HardwareIO, LimelightVisionInputs> limelightWithInput : limelightsWithInputsList){
      Limelight4HardwareIO limelight = limelightWithInput.getFirst();
      LimelightVisionInputs inputs = limelightWithInput.getSecond();

      limelight.updateInputs(inputs);
      Logger.processInputs(kLogPrefixInput + "/" + limelight.getConfiguration().toString(), inputs);
      limelight.updateLog(kLogPrefixStandard, kLogPrefixInput);

      // If this input has a valid robot pose we want to add it to our observation list
      if(inputs.hasTag){
        Optional<VisionFieldPoseEstimate> visionEstimate = LimelightIO.proccessVisionInputs(
          inputs, 
          limelight, 
          RobotRuntimeConstants.kCurrentYear,
          () -> RobotState.get().getLastUsedMegatagTimestamp(),
          (timestamp) -> RobotState.get().getFieldRobotPose(timestamp),
          (min, max) -> RobotState.get().getMaxDriveYawSpeedInRange(min, max)
        );
        if(visionEstimate.isPresent()){
          limelightFieldPoseEstimates.add(visionEstimate.get());
        }
      }
    }

    Optional<VisionFieldPoseEstimate> accepted = Optional.empty();
    if(limelightFieldPoseEstimates.size() > 0){
      accepted = LimelightIO.fuseEstimates(
          (timestamp) -> RobotState.get().getFieldRobotPose(timestamp), 
          limelightFieldPoseEstimates
        );
    }

    accepted.ifPresent(
      (acc) -> RobotState.get().addMegatagEstimateMeasurement(acc)
    );
    
    updateLog();
  }

  

  

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(standardPrefix + "/EstimatedMegatagMeasurement",  RobotState.get().getLatestMegatagEstimatedPose());
  }
}
