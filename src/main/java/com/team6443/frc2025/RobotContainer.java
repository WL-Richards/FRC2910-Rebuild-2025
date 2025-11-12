// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team6443.frc2025.commands.drive.DriveWithHeadingCommand;
import com.team6443.frc2025.subsystems.SubsystemFactory;
import com.team6443.frc2025.subsystems.drive.DrivetrainSubsystem;
import com.team6443.frc2025.subsystems.elevator.ElevatorSubsystem;
import com.team6443.lib.config.motors.ServoMotorCANCoderConfiguration;
import com.team6443.lib.controllers.XboxControllerImplementation;
import com.team6443.lib.controllers.interfaces.DriveControllable;
import com.team6443.lib.logging.interfaces.Loggerable;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer implements Loggerable {

  private final DriveControllable driveController = new XboxControllerImplementation(0);

  // --- Elevator subsystem ---
  private final ElevatorSubsystem elevatorSubsystem = SubsystemFactory.createElevatorSubsystem(); 

  // --- Drive train system ---
  private final DrivetrainSubsystem drivetrainSubsystem = SubsystemFactory.createDrivetrainSubsystem();
  private final DriveWithHeadingCommand drivetrainDefaultCommand = new DriveWithHeadingCommand(
    drivetrainSubsystem, 
    driveController::getThrottle,   // throttle
    driveController::getStrafe,     // strafe
    driveController::getRotation    // turn
  );

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Set default drive train command
    drivetrainSubsystem.setDefaultCommand(drivetrainDefaultCommand);
  }
}
