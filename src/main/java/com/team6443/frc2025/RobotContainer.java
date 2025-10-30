// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team6443.frc2025.subsystems.SubsystemFactory;
import com.team6443.frc2025.subsystems.TestSubsystem;
import com.team6443.frc2025.subsystems.drive.DrivetrainSubsystem;
import com.team6443.frc2025.subsystems.elevator.ElevatorSubsystem;
import com.team6443.lib.config.motors.ServoMotorCANCoderConfiguration;
import com.team6443.lib.logging.interfaces.Loggerable;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer implements Loggerable {

  private final ElevatorSubsystem elevatorSubsystem = SubsystemFactory.createElevatorSubsystem(); 
  private final DrivetrainSubsystem drivetrainSubsystem = SubsystemFactory.creatDrivetrainSubsystem();

  private CommandXboxController primary = new CommandXboxController(0);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    primary.axisGreaterThan(0, 0).onTrue(drivetrainSubsystem.applyRequest(() -> new SwerveRequest.FieldCentric().withVelocityY(-5)));
    primary.axisLessThan(0, 0.001).onTrue(drivetrainSubsystem.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));
  }
}
