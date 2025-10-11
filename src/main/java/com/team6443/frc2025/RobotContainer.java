// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025;
import com.team6443.lib.logging.interfaces.Loggerable;

public class RobotContainer implements Loggerable {

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  @Override
  public void updateLogger() {
    // Update the robot state log
    RobotState.get().updateLog();
  }

}
