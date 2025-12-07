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
import com.team6443.lib.autonomous.ChoreoTrajectoryCommandFactory;
import com.team6443.lib.autonomous.LoggableAutoTrajectory;
import com.team6443.lib.input.XboxInputImplementation;
import com.team6443.lib.logging.interfaces.Loggerable;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final ChoreoPathing choreoPathing = SubsystemFactory.createChoreoPathing(drivetrainSubsystem);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Set default drive train command
    drivetrainSubsystem.setDefaultCommand(drivetrainDefaultCommand);
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

  private void setupAutos(AutoChooser autoChooser){
     // Add options to the chooser
     autoChooser.addRoutine("Test Auto Routine", this::testAutoRoutine);
    
  }

  public AutoRoutine testAutoRoutine(){
    AutoRoutine routine = choreoPathing.getAutoFactory().newRoutine("testRoutine");

    // Create a new auto trajectory that will update the active trajecotry in the choreoPathing so we can log the current running trajectory
    LoggableAutoTrajectory testTraj = ChoreoTrajectoryCommandFactory.createTrajectoryForRoutine(
      choreoPathing, routine, "TestPath");

    // When the routine begins, reset odometry and start the first trajectory 
    routine.active().onTrue(
      Commands.sequence(
          testTraj.resetOdometry(),
          testTraj.cmd()
      )
    );

    return routine;
  }
}
