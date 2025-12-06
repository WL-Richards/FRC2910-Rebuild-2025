// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025;

import org.littletonrobotics.junction.LoggedRobot;

import com.team6443.lib.RobotState;
import com.team6443.lib.SimulatedRobotState;
import com.team6443.lib.can.CANStatusLogger;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends LoggedRobot {
  private final RobotContainer robotContainer;

  public Robot() {
    // SHOULD ALWAYS BE CALLED FIRST TO NOT MISS ANY LOGS
    // Setup logging to the proper location, and log the metadata for the bot
    robotContainer = new RobotContainer();
    robotContainer.setupLogger();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    // Update the robot state
    RobotState.get().updateLog();

    // CAN updates can be run here, and will be throttled to improve robot performance
    //CANStatusLogger.updateAllLogs();
  }

  @Override
  public void simulationPeriodic() {
    SimulatedRobotState.get().updateState();
    SimulatedRobotState.get().updateLog();
  }

  @Override
  public void disabledInit() {
    CANStatusLogger.SetRobotDisabled();     // Tell the can logger the robot is now disabled and it can update faster
  }

  @Override
  public void disabledPeriodic() {

    // But im still just gonna run them here for now
    CANStatusLogger.updateAllLogs();
  }

  @Override
  public void disabledExit() {
    CANStatusLogger.SetRobotEnabled();    // Tell the can logger the robot is now enabled and we should update the logs slower
  }

  @Override
  public void autonomousInit() {
    robotContainer.createTestPath().schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() { 
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
