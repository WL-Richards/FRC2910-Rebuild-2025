// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.team6443.frc2025.state.RobotState;
import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.subsystems.AEMSubsystem;
import com.team6443.lib.subsystems.vision.VisionInputs;
import com.team6443.lib.subsystems.vision.interfaces.CameraIO;
import com.team6443.lib.subsystems.vision.io.limelight.Limelight4HardwareIO;

import edu.wpi.first.math.Pair;


public class VisionSubsystem extends AEMSubsystem {

  // Limelight object with its corresponding inputs
  private final List<Pair<Limelight4HardwareIO, VisionInputs>> limelightsWithInputsList = new ArrayList<>();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(Limelight4HardwareIO... limelights) {
    super("VisionSubsystem");

    for (Limelight4HardwareIO limelight : limelights){
      limelightsWithInputsList.add(Pair.of(limelight, new VisionInputs()));
    }
  }

  @Override
  public void periodic() {

    /**
     * For every limelight update its inputs and also determine if it had a valid pose that needs to be recorded
     */
    for (Pair<Limelight4HardwareIO, VisionInputs> limelightWithInput : limelightsWithInputsList){
      Limelight4HardwareIO limelight = limelightWithInput.getFirst();
      VisionInputs inputs = limelightWithInput.getSecond();

      limelight.updateInputs(inputs);
      Logger.processInputs(kLogPrefixInput + "/" + limelight.getConfiguration().toString(), inputs);
      limelight.updateLog(kLogPrefixStandard, kLogPrefixInput);

      // If this input has a valid robot pose we want to add it to our observation list
      if(inputs.hasTag){
        updateEstimationFromVision(inputs, limelight);
      }
    }
  }

  private void updateEstimationFromVision(VisionInputs inputs, CameraIO camera){

  }

  

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
  }
}
