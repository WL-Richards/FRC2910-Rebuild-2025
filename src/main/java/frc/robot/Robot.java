// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.RobotRuntimeConstants;
import frc.robot.util.can.CANDeviceID;
import frc.robot.util.can.CANStatusLogger;
import frc.robot.util.motors.TalonFXFactory;

public class Robot extends LoggedRobot {
  private final RobotContainer m_robotContainer;

  public Robot() {
    // SHOULD ALWAYS BE CALLED FIRST TO NOT MISS ANY LOGS
    // Setup logging to the proper location, and log the metadata for the bot
    m_robotContainer = new RobotContainer();
    m_robotContainer.setupLogger();
    TalonFX testTalonFX = TalonFXFactory.createDefault(
      new CANDeviceID(
        0, 
        "TestTalonDevice", 
        CANDeviceID.CANDeviceType.TALON_FX, 
        CANStatusLogger.get(0).getBusName()
      )
    );
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    m_robotContainer.updateLogger();
  }

  @Override
  public void simulationInit() {
      super.simulationInit();
  }

  @Override
  public void simulationPeriodic() {
      // TODO Auto-generated method stub
      super.simulationInit();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {

    // Only update CAN status logging when disabled to save on performance
    CANStatusLogger.updateAllLogs();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

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
