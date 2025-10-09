// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.interfaces.logging.Loggerable;

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
