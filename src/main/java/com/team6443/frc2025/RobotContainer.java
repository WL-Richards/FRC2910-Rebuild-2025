// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025;
import org.littletonrobotics.junction.LoggedRobot;

import com.team6443.frc2025.autonomous.AutoCommandFactory;
import com.team6443.frc2025.autonomous.AutoRoutineFactory;
import com.team6443.frc2025.constants.RobotRuntimeConstants;
import com.team6443.frc2025.subsystems.SubsystemFactory;
import com.team6443.frc2025.subsystems.drive.DrivetrainSubsystem;
import com.team6443.frc2025.subsystems.drive.commands.DriveWithHeadingCommand;
import com.team6443.frc2025.subsystems.elevator.ElevatorSubsystem;
import com.team6443.frc2025.subsystems.vision.VisionSubsystem;
import com.team6443.lib.autonomous.ChoreoPather;
import com.team6443.lib.autonomous.wrappers.LoggableAutoTrajectory;
import com.team6443.lib.autonomous.wrappers.LoggableChoreoCommand;
import com.team6443.lib.core.logging.Loggerable;
import com.team6443.lib.input.XboxInputImplementation;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

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
  private final ChoreoPather choreoPather = SubsystemFactory.createChoreoPather(drivetrainSubsystem);

  // Factories for generating poth AutoRoutines and auto commands
  private final AutoCommandFactory autoCommandFactory = new AutoCommandFactory(choreoPather);
  private final AutoRoutineFactory autoRoutineFactory = new AutoRoutineFactory(choreoPather);
  
  public RobotContainer(LoggedRobot robot) {
    setupLogger(robot);
    configureBindings();
    setupAutoChooser();

    RobotRuntimeConstants.kRobotConfiguration.getSuperstructureConfiguration().generateSuperstructureGraphTemplate();
    RobotRuntimeConstants.kRobotConfiguration.getSuperstructureConfiguration().loadGraph();
  }

  /**
   * Setup the auto chooser with the desired auto paths
   */
  public void setupAutoChooser(){

    // Create the auto chooser
    AutoChooser autoChooser = new AutoChooser();

    // Add all the routines or commands to the chooser
    this.setupAutos(autoChooser);
   
    // Put the auto chooser on the dashboard
    SmartDashboard.putData("AutoChooser", autoChooser);

    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  /**
   * Configure all the auto routes that can be chosen with the AutoChooser
   * @param autoChooser The AutoChooser being populated
   */
  private void setupAutos(AutoChooser autoChooser){
    // Add options to the chooser
    autoChooser.addCmd("Test Auto Command", autoCommandFactory::testAutoCommand);
    autoChooser.addRoutine("Test Auto Routine", autoRoutineFactory::testAutoRoutine);
  }

  private void configureBindings() {
    // Set default drive train command
    drivetrainSubsystem.setDefaultCommand(drivetrainDefaultCommand);
  }

 
}
