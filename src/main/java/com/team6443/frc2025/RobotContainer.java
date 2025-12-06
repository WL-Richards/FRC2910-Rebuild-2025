// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025;
import com.team6443.frc2025.commands.drive.DriveWithHeadingCommand;
import com.team6443.frc2025.constants.RobotRuntimeConstants;
import com.team6443.frc2025.subsystems.SubsystemFactory;
import com.team6443.frc2025.subsystems.drive.DrivetrainSubsystem;
import com.team6443.frc2025.subsystems.elevator.ElevatorSubsystem;
import com.team6443.frc2025.subsystems.vision.VisionSubsystem;
import com.team6443.lib.autonomous.ChoreoPathing;
import com.team6443.lib.input.XboxInputImplementation;
import com.team6443.lib.logging.interfaces.Loggerable;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer implements Loggerable {

  // --- Robot inputs ---
  private final XboxInputImplementation primaryController = new XboxInputImplementation(0);

  // --- Elevator subsystem ---
  private final ElevatorSubsystem elevatorSubsystem = SubsystemFactory.createElevatorSubsystem(); 

  // --- Drive train system ---
  private final DrivetrainSubsystem drivetrainSubsystem = SubsystemFactory.createDrivetrainSubsystem();
  private final DriveWithHeadingCommand drivetrainDefaultCommand = new DriveWithHeadingCommand(
    drivetrainSubsystem,
    RobotRuntimeConstants.kRobotConfiguration.getDrivetrainConfiguration(),
    primaryController::getThrottle,   // throttle
    primaryController::getStrafe,     // strafe
    primaryController::getRotation   // turn
    
  );

  // --- Vision system ---
  private final VisionSubsystem visionSubsystem = SubsystemFactory.createVisionSubsystem();

  // --- Auto Factory ---
  private final ChoreoPathing choreoPathing = SubsystemFactory.createChoreoPathing(drivetrainSubsystem);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Set default drive train command
    drivetrainSubsystem.setDefaultCommand(drivetrainDefaultCommand);
  }

  public Command createTestPath(){
    return Commands.sequence(
      choreoPathing.getAutoFactory().resetOdometry("TestPath"),
      choreoPathing.getAutoFactory().trajectoryCmd("TestPath")
    );
  }
}
